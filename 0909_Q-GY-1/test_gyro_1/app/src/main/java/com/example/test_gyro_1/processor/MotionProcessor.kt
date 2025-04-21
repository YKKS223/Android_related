package com.example.test_gyro_1.processor

import android.graphics.PointF
import android.hardware.SensorEvent
import android.util.Log
import com.example.test_gyro_1.filter.ExtendedKalmanFilter
import com.example.test_gyro_1.gps.LocationData
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class MotionProcessor {

    //EKF関連
    private val ekf = ExtendedKalmanFilter()
    //QuaternionProcessor
    private val quaternionProcessor = QuaternionProcessor()

    //初期角度を保持する変数
    //[Yaw, Pitch, Roll]の初期絶対角度
    private var initialEulerAnglesDeg: FloatArray? = null
    //計測が開始され、初期角度が設定されたかを示すフラグ
    private var isMeasurementStarted = false
    //回転ベクトルデータが少なくとも1回は受信されたかを示すフラグ
    private var hasRotationVector = false

    //状態変数
    private val worldAccel = FloatArray(3) //ワールド座標系での加速度
    //rotationMatrix は QuaternionProcessor から取得するので、ここで保持する必要はなくなった
    //private val rotationMatrix = FloatArray(9).apply { this[0] = 1f; this[4] = 1f; this[8] = 1f }

    private var accelLastTimestamp: Long = 0
    private var isAccelFirstTime = true
    //現在のジャイロレート(静止判定用)
    private var currentGyroRate = FloatArray(3)
    private var isStationary = false
    private var isHighSpeedMode = false
    private var latestGpsLocationData: LocationData? = null
    private var lastValidGpsData: LocationData? = null
    private var lastEkfPositionAtGpsUpdate: FloatArray? = null

    //軌跡履歴用
    private val positionHistory = mutableListOf<PointF>()
    private var lastPositionAddedToHistory: PointF? = null
    private val MIN_DISTANCE_TO_ADD_HISTORY = 0.1f // 0.1m以上動いたら履歴に追加 (bf 0.5から変更)

    //移動総距離計算用
    private var accumulatedDistance: Float = 0f
    private var lastDistanceAccumulationTime: Long = 0L
    private var lastPositionAtAccumulation: FloatArray? = null

    //gyroLastTimestamp, isGyroFirstTime は QuaternionProcessor が回転を扱うため不要

    //設定可能な定数
    companion object {
        private const val ACCUMULATION_INTERVAL_SECONDS = 1.0f // 1秒ごとに距離を加算試行 (bf 5.0)
        private const val ACCUMULATION_INTERVAL_MS = (ACCUMULATION_INTERVAL_SECONDS * 1000).toLong()
        private const val MIN_ACCUMULATION_DISTANCE_THRESHOLD = 0.05f // 5cm以上の移動で加算 (bf 0.50)
        private const val MAX_ACCUMULATION_STEP_DISTANCE = 10.0f // 1回の加算ステップでの最大距離 (異常値対策)
        //閾値
        private const val stationaryAccelThreshold = 0.08f // 静止判定の加速度閾値 (m/s^2) (bf 0.05)
        private const val stationaryGyroThreshold = 0.03f // 静止判定のジャイロ閾値 (rad/s) (bf 0.05)
        private const val MIN_VELOCITY_THRESHOLD = 0.1f   // これ以下の速度はクリップ (m/s)
        private const val GPS_CONSISTENCY_DISTANCE_THRESHOLD = 2.0f // EKFとGPSの矛盾検出閾値(EKF移動量)
        private const val GPS_STATIONARY_DISTANCE_RATIO = 0.75f // GPS精度に対する静止移動量の割合
        private const val HIGH_SPEED_THRESHOLD_KMH = 5.0f // 高速モードに入る速度 (km/h)
        private const val HIGH_SPEED_THRESHOLD_MPS = HIGH_SPEED_THRESHOLD_KMH / 3.6f // m/s
    }

    //UIへ状態を通知するための StateFlow
    private val _motionStateFlow = MutableStateFlow(MotionState.initial())
    val motionStateFlow: StateFlow<MotionState> = _motionStateFlow.asStateFlow()

    //リセット
    fun reset() {
        Log.i("MotionProcessor", "Resetting MotionProcessor state...")
        ekf.reset()
        quaternionProcessor.reset() //QuaternionProcessor もリセット

        //状態フラグと初期角度をリセット
        initialEulerAnglesDeg = null
        isMeasurementStarted = false
        hasRotationVector = false

        worldAccel.fill(0f)
        accelLastTimestamp = 0L
        isAccelFirstTime = true
        currentGyroRate.fill(0f)
        isStationary = false
        isHighSpeedMode = false
        latestGpsLocationData = null
        lastValidGpsData = null
        lastEkfPositionAtGpsUpdate = null

        //総距離・軌跡履歴のリセット
        accumulatedDistance = 0f
        lastDistanceAccumulationTime = 0L
        lastPositionAtAccumulation = null
        positionHistory.clear()
        lastPositionAddedToHistory = null

        //StateFlow を初期状態に更新
        //initial() は角度0なので、リセット直後の表示に適している
        _motionStateFlow.value = MotionState.initial()
        Log.i("MotionProcessor", "Processor reset complete.")
    }

    fun processGyroscopeEvent(event: SensorEvent) {
        // 静止判定などで使うため現在の角速度を保持
        currentGyroRate[0] = event.values[0]
        currentGyroRate[1] = event.values[1]
        currentGyroRate[2] = event.values[2]
    }

    fun processRotationVectorEvent(event: SensorEvent) {
        //QuaternionProcessor に処理を委譲して絶対角度を計算させる
        quaternionProcessor.processRotationVectorEvent(event)
        //回転ベクトルデータを受け取ったことを示すフラグ
        if (!hasRotationVector) {
            Log.i("MotionProcessor", "First rotation vector event received.")
        }
        hasRotationVector = true

        //まだ計測が開始されておらず、有効な回転ベクトルデータがあれば初期角度を設定
        //isMeasurementStarted フラグで初回のみ実行されるようにする
        if (!isMeasurementStarted && hasRotationVector) {
            // QuaternionProcessor から現在の絶対角度 [Yaw, Pitch, Roll] を取得
            val currentAbsoluteEulerAngles = quaternionProcessor.getEulerAnglesDegrees()

            //初期角度として保存（clone() を忘れない)
            initialEulerAnglesDeg = currentAbsoluteEulerAngles.clone()
            isMeasurementStarted = true //計測開始フラグを立てる
            Log.i("MotionProcessor", "Initial orientation set (Yaw=%.1f, Pitch=%.1f, Roll=%.1f)".format(
                initialEulerAnglesDeg!![0], initialEulerAnglesDeg!![1], initialEulerAnglesDeg!![2]))

            //重要: 初期角度(相対角度0)を即座にUIに反映させるために updateMotionState を呼ぶ
            //これにより、加速度センサーが動き出す前でも角度表示が0になる
            updateMotionState()
        }
    }


    fun processLinearAccelerationEvent(event: SensorEvent) {
        //初期角度が設定されるまで (isMeasurementStarted=true になるまで) は処理しない
        if (!isMeasurementStarted) {
            //Log.d("MotionProcessor", "Skipping acceleration event: Measurement not started (initial orientation not set).")
            return
        }

        val currentTimestamp = event.timestamp
        //最初のイベント、またはタイムスタンプがリセットされた場合
        if (isAccelFirstTime || accelLastTimestamp == 0L) {
            accelLastTimestamp = currentTimestamp
            isAccelFirstTime = false
            return
        }

        //時間差分 dt (秒) を計算
        val dt = (currentTimestamp - accelLastTimestamp) * 1e-9f // (1.0f / 1000000000.0f)
        //dt が非常に小さい、または負の場合はスキップ（タイムスタンプの異常）
        if (dt <= 1e-7f) {
            Log.w("MotionProcessor", "Skipping acceleration event: Invalid dt = %.9f".format(dt))
            //タイムスタンプは更新しておく
            accelLastTimestamp = currentTimestamp
            return
        }
        accelLastTimestamp = currentTimestamp

        //ワールド座標系での加速度を計算するために、最新の回転行列を取得
        val currentRotationMatrix = quaternionProcessor.getRotationMatrix()
        if (currentRotationMatrix == null) {
            Log.w("MotionProcessor", "Skipping acceleration event: Rotation matrix not available from QuaternionProcessor.")
            return //回転行列がまだ準備できていない場合は処理中断
        }

        //デバイス座標系の線形加速度
        val devAccX = event.values[0]
        val devAccY = event.values[1]
        val devAccZ = event.values[2]

        //ワールド座標系の加速度に変換
        worldAccel[0] = currentRotationMatrix[0] * devAccX + currentRotationMatrix[1] * devAccY + currentRotationMatrix[2] * devAccZ
        worldAccel[1] = currentRotationMatrix[3] * devAccX + currentRotationMatrix[4] * devAccY + currentRotationMatrix[5] * devAccZ
        worldAccel[2] = currentRotationMatrix[6] * devAccX + currentRotationMatrix[7] * devAccY + currentRotationMatrix[8] * devAccZ

        //ここから EKF 処理、静止判定、距離計算など
        //(静止判定ロジック - 変更なし)
        val accelMagnitude = sqrt(worldAccel[0].pow(2) + worldAccel[1].pow(2) + worldAccel[2].pow(2))
        val gyroMagnitude = sqrt(currentGyroRate[0].pow(2) + currentGyroRate[1].pow(2) + currentGyroRate[2].pow(2))
        val currentGpsSpeedMps = latestGpsLocationData?.speed ?: 0.0f
        val isGpsSpeedBelowThreshold = currentGpsSpeedMps < HIGH_SPEED_THRESHOLD_MPS
        val isImuStationary = accelMagnitude < stationaryAccelThreshold && gyroMagnitude < stationaryGyroThreshold
        val isConsideredStationary = isImuStationary && isGpsSpeedBelowThreshold
        val justBecameStationary = !isStationary && isConsideredStationary
        val isImuNearlyStationary = accelMagnitude < stationaryAccelThreshold * 1.5f && gyroMagnitude < stationaryGyroThreshold * 1.5f
        val remainsStationary = isStationary && isImuNearlyStationary && isGpsSpeedBelowThreshold

        if (justBecameStationary || remainsStationary) {
            if (!isStationary) {
                //Log.i("MotionProcessor", "Detected stationary state (IMU & GPS criteria met)")
                isStationary = true
            }
            ekf.updateStationary() //静止状態のEKF更新 (速度を0に近づける)
            ekf.predict(dt, floatArrayOf(0f, 0f, 0f)) // 静止時は加速度0で予測
        } else {
            if (isStationary) {
                //Log.i("MotionProcessor", "Exited stationary state")
                isStationary = false
            }
            ekf.predict(dt, worldAccel) // 観測された加速度でEKF予測
        }

        //EKF更新前の位置を保持 (GPS矛盾チェック用)
        val ekfPositionBeforeGpsUpdate = ekf.getPosition().clone()

        //GPSデータによるEKF更新 (ロジック変更なし)
        val gpsDataToUse = latestGpsLocationData
        if (gpsDataToUse != null && gpsDataToUse.isLocalValid) {
            //Log.d("MotionProcessor", "Attempting EKF update with GPS data (Timestamp: ${gpsDataToUse.timestamp}, HighSpeed: $isHighSpeedMode)")
            val updateSuccessful = try {
                ekf.updateGpsPosition(gpsDataToUse, isHighSpeedMode)
                true
            } catch (e: Exception) {
                Log.e("MotionProcessor", "Error during EKF GPS update", e)
                false
            }

            //GPS更新後の矛盾チェック (成功した場合のみ)
            if (updateSuccessful) {
                val lastGps = lastValidGpsData
                val lastEkfPos = lastEkfPositionAtGpsUpdate
                //(矛盾補正処理 - 元のコードを踏襲)
                if (lastGps != null && lastGps.isLocalValid && lastEkfPos != null &&
                    gpsDataToUse.localX != null && gpsDataToUse.localY != null &&
                    lastGps.localX != null && lastGps.localY != null) {

                    val dxGps = gpsDataToUse.localX - lastGps.localX
                    val dyGps = gpsDataToUse.localY - lastGps.localY
                    val distGps = sqrt(dxGps * dxGps + dyGps * dyGps)

                    val dxEkf = ekfPositionBeforeGpsUpdate[0] - lastEkfPos[0]
                    val dyEkf = ekfPositionBeforeGpsUpdate[1] - lastEkfPos[1]
                    val distEkf = sqrt(dxEkf * dxEkf + dyEkf * dyEkf)

                    //条件: EKFの移動量がGPSの移動量より著しく大きい場合 (元のコードの条件に近い形で)
                    //(GPSがほとんど動いていないのにEKFが大きく動いた場合など)
                    val inconsistencyRatioThreshold = 3.0f
                    val inconsistencyMinEkfDistance = 1.0f

                    if (distGps > 1e-3f && distEkf > inconsistencyMinEkfDistance && distEkf / distGps > inconsistencyRatioThreshold) {
                        Log.w("MotionProcessor", "Inconsistency detected! EKF moved %.2fm, GPS moved %.2fm (Ratio %.1f > %.1f). Correcting EKF position.".format(distEkf, distGps, distEkf / distGps, inconsistencyRatioThreshold))
                        val correctionFactor = 0.4f
                        val correctedEkfX = lastEkfPos[0] + dxEkf * correctionFactor
                        val correctedEkfY = lastEkfPos[1] + dyEkf * correctionFactor
                        ekf.x[0, 0] = correctedEkfX
                        ekf.x[1, 0] = correctedEkfY
                        Log.w("MotionProcessor", "Corrected EKF pos: X=%.2f, Y=%.2f".format(ekf.x[0, 0], ekf.x[1, 0]))
                    }
                }
                lastValidGpsData = gpsDataToUse.copy()
                lastEkfPositionAtGpsUpdate = ekfPositionBeforeGpsUpdate
            }
        }

        //低速度域での速度クリッピング (変更なし、静止状態でない場合のみ適用)
        val currentVelocity = ekf.getVelocity()
        val currentSpeed = sqrt(currentVelocity[0].pow(2) + currentVelocity[1].pow(2) + currentVelocity[2].pow(2))
        if (!isStationary && currentSpeed < MIN_VELOCITY_THRESHOLD) {
            if (ekf.x[3, 0] != 0f || ekf.x[4, 0] != 0f || ekf.x[5, 0] != 0f) {
                //Log.d("MotionProcessor", "Clipping speed below threshold (%.2f m/s). Speed: %.4f m/s".format(MIN_VELOCITY_THRESHOLD, currentSpeed))
                ekf.x[3, 0] = 0f; ekf.x[4, 0] = 0f; ekf.x[5, 0] = 0f
            }
        }

        //EKF更新後の最新位置を取得
        val currentPositionArray = ekf.getPosition()
        val currentPointF = PointF(currentPositionArray[0], currentPositionArray[1]) // XY平面

        //移動総距離の計算 (変更なし)
        val currentTime = System.currentTimeMillis()
        val lastPosArray = lastPositionAtAccumulation
        if (lastPosArray != null) {
            if (currentTime - lastDistanceAccumulationTime >= ACCUMULATION_INTERVAL_MS) {
                val dx = currentPositionArray[0] - lastPosArray[0]
                val dy = currentPositionArray[1] - lastPosArray[1]
                val dz = currentPositionArray[2] - lastPosArray[2]
                val deltaDistance = sqrt(dx.pow(2) + dy.pow(2) + dz.pow(2))

                if (deltaDistance >= MIN_ACCUMULATION_DISTANCE_THRESHOLD && deltaDistance <= MAX_ACCUMULATION_STEP_DISTANCE) {
                    accumulatedDistance += deltaDistance
                    //Log.d("MotionProcessor", "Accumulated distance: +%.2fm (Total: %.2fm)".format(deltaDistance, accumulatedDistance))
                } else if (deltaDistance > MAX_ACCUMULATION_STEP_DISTANCE) {
                    Log.w("MotionProcessor", "Skipping large distance accumulation step: %.2f m (Threshold: %.1f m)".format(deltaDistance, MAX_ACCUMULATION_STEP_DISTANCE))
                } //else: 閾値未満は加算しない

                lastDistanceAccumulationTime = currentTime
                lastPositionAtAccumulation = currentPositionArray.clone()
            }
        } else {
            lastPositionAtAccumulation = currentPositionArray.clone()
            lastDistanceAccumulationTime = currentTime
        }

        //軌跡履歴への追加 (変更なし)
        val lastAddedPointF = lastPositionAddedToHistory
        var shouldAddToHistory = false
        if (lastAddedPointF == null) {
            shouldAddToHistory = true
        } else {
            val distFromLast = sqrt((currentPointF.x - lastAddedPointF.x).pow(2) + (currentPointF.y - lastAddedPointF.y).pow(2))
            if (distFromLast >= MIN_DISTANCE_TO_ADD_HISTORY) {
                shouldAddToHistory = true
            }
        }
        if (shouldAddToHistory) {
            positionHistory.add(currentPointF)
            lastPositionAddedToHistory = currentPointF
        }

        //EKF処理、距離計算などここまで

        //最後に状態を更新してUIに通知
        updateMotionState()
    }

    fun processGpsData(locationData: LocationData?) {
        this.latestGpsLocationData = locationData // 最新のGPSデータを保持

        if (locationData != null) {
            //高速移動モードの更新ロジック (変更なし)
            val currentGpsSpeed = locationData.speed
            val isValidAndAccurate = locationData.isLocalValid && (locationData.accuracy ?: Float.MAX_VALUE) < 30f

            if (currentGpsSpeed != null && isValidAndAccurate) {
                val enteringHighSpeed = currentGpsSpeed > HIGH_SPEED_THRESHOLD_MPS && !isHighSpeedMode
                val exitingHighSpeed = currentGpsSpeed <= HIGH_SPEED_THRESHOLD_MPS && isHighSpeedMode

                if (enteringHighSpeed) {
                    Log.i("MotionProcessor", "Entered High Speed Mode (GPS Speed: %.2f m/s)".format(currentGpsSpeed))
                    isHighSpeedMode = true
                    //高速モード移行時にEKFのGPS信頼度を上げる処理をここで行うことも検討可能
                    //(現在は updateGpsPosition 内で isHighSpeedMode フラグを見て判断している)
                } else if (exitingHighSpeed) {
                    Log.i("MotionProcessor", "Exited High Speed Mode (GPS Speed: %.2f m/s)".format(currentGpsSpeed))
                    isHighSpeedMode = false
                }
            } else {
                if (isHighSpeedMode) {
                    Log.i("MotionProcessor", "Exited High Speed Mode (Invalid GPS speed or low accuracy)")
                    isHighSpeedMode = false
                }
            }
        } else {
            if (isHighSpeedMode) {
                Log.i("MotionProcessor", "Exited High Speed Mode (GPS data became null)")
                isHighSpeedMode = false
            }
        }
        //GPSデータ受信時にも状態を更新するか？ -> 加速度イベントで十分頻繁に更新されるため、通常は不要
        //updateMotionState()
    }

    private fun updateMotionState() {
        //EKFから最新の状態を取得
        val currentState = ekf.getState()
        val position = floatArrayOf(currentState[0], currentState[1], currentState[2])
        val velocity = floatArrayOf(currentState[3], currentState[4], currentState[5])
        val speedMps = sqrt(velocity[0].pow(2) + velocity[1].pow(2) + velocity[2].pow(2))
        val speedKmh = speedMps * 3.6f
        //totalDistance は原点からの直線距離(変位)
        val totalDistance = sqrt(position[0].pow(2) + position[1].pow(2) + position[2].pow(2))

        //相対角度の計算
        val relativeYaw: Float
        val relativePitch: Float
        val relativeRoll: Float

        //isMeasurementStarted フラグと initialEulerAnglesDeg の null チェックを行う
        if (isMeasurementStarted && initialEulerAnglesDeg != null) {
            //QuaternionProcessorから現在の絶対オイラー角 [Yaw, Pitch, Roll] を取得
            val currentAbsoluteEulerAngles = quaternionProcessor.getEulerAnglesDegrees()

            //差分を計算
            val diffYaw = currentAbsoluteEulerAngles[0] - initialEulerAnglesDeg!![0]
            val diffPitch = currentAbsoluteEulerAngles[1] - initialEulerAnglesDeg!![1]
            val diffRoll = currentAbsoluteEulerAngles[2] - initialEulerAnglesDeg!![2]

            //角度を正規化 (-180 ~ +180 の範囲に)
            relativeYaw = normalizeAngle(diffYaw)
            relativePitch = normalizeAngle(diffPitch) // Pitch も正規化
            relativeRoll = normalizeAngle(diffRoll)

        } else {
            //初期角度がまだ設定されていない場合は 0 とする
            //(reset 直後や、最初の回転ベクトルデータ受信前など)
            relativeYaw = 0f
            relativePitch = 0f
            relativeRoll = 0f
        }
        //ここまで相対角度計算

        //状態表示テキストと色を決定 (変更なし)
        val statusText: String
        val statusColor: Int //android.R.color.xxx を使用
        when {
            isStationary -> {
                statusText = "状態: 静止"
                statusColor = android.R.color.holo_green_dark
            }
            isHighSpeedMode -> {
                statusText = "状態: 高速移動 (GPS優先)"
                statusColor = android.R.color.holo_red_light
            }
            speedMps < MIN_VELOCITY_THRESHOLD && !isStationary -> {
                statusText = "状態: 低速" //(速度クリップ中)は詳細すぎるかも
                statusColor = android.R.color.holo_blue_dark
            }
            else -> { //移動中 (静止でも高速でも低速でもない)
                statusText = "状態: 移動中"
                statusColor = android.R.color.holo_orange_dark
            }
        }

        //新しい MotionState を作成
        val newState = MotionState(
            position = position,
            velocity = velocity,
            worldAccel = worldAccel.clone(),
            //相対角度をセット
            angleYaw = relativeYaw,
            anglePitch = relativePitch,
            angleRoll = relativeRoll,

            isStationary = this.isStationary,
            isHighSpeedMode = this.isHighSpeedMode,
            speedMps = speedMps,
            speedKmh = speedKmh,
            totalDistance = totalDistance, // 原点からの直線距離
            accumulatedDistance = this.accumulatedDistance, // 移動総距離
            currentPoint = PointF(position[0], position[1]), // XY平面の現在位置
            pathHistory = positionHistory.toList(), // 軌跡履歴 (不変リストとしてコピー)
            statusText = statusText,
            statusColor = statusColor,
            latestGpsData = this.latestGpsLocationData?.copy() // GPSデータもコピー
        )

        //StateFlow の値を更新 (現在の値と異なる場合のみ)
        if (_motionStateFlow.value != newState) {
            _motionStateFlow.value = newState
        }
    }

    private fun normalizeAngle(angle: Float): Float {
        var normalizedAngle = angle % 360.0f
        //結果が (-360, 360) の範囲になる
        if (normalizedAngle > 180.0f) {
            normalizedAngle -= 360.0f // (180, 360) -> (-180, 0)
        } else if (normalizedAngle <= -180.0f) {
            //(-360, -180] -> (0, 180]
            //<= -180 の条件にすることで -180 は +180 になる
            normalizedAngle += 360.0f
        }
        //結果は (-180, 180] の範囲になるはず
        return normalizedAngle
    }

    //オプション: 外部から最新の回転行列を取得したい場合
    fun getLatestRotationMatrix(): FloatArray? {
        return quaternionProcessor.getRotationMatrix()
    }
}