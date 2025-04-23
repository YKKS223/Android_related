package com.example.test_gyro_1.processor

import android.graphics.PointF
import android.hardware.SensorEvent
import android.util.Log
import com.example.test_gyro_1.filter.AttitudeEKF // ★ Import
import com.example.test_gyro_1.filter.ExtendedKalmanFilter
import com.example.test_gyro_1.gps.LocationData
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.*
import android.hardware.SensorManager

class MotionProcessor {

    private val positionEKF = ExtendedKalmanFilter() //位置・速度EKF (名前変更推奨)
    private val attitudeEKF = AttitudeEKF() //姿勢EKF

    //状態変数
    //private var isMeasurementStarted = false //attitudeEKF が初期化されたかで判断可能
    private val worldAccel = FloatArray(3)
    private var accelLastTimestamp: Long = 0
    private var isAccelFirstTime = true
    private var currentGyroRate = FloatArray(3) // 姿勢EKFへの入力 & 静止判定用
    private var isStationary = false
    private var isHighSpeedMode = false
    private var latestGpsLocationData: LocationData? = null

    private var lastValidGpsData: LocationData? = null
    private var lastEkfPositionAtGpsUpdate: FloatArray? = null
    private val positionHistory = mutableListOf<PointF>()
    private var lastPositionAddedToHistory: PointF? = null
    private var accumulatedDistance: Float = 0f
    private var lastDistanceAccumulationTime: Long = 0L
    private var lastPositionAtAccumulation: FloatArray? = null

    //姿勢関連
    private var currentAttitudeQuaternion: FloatArray = floatArrayOf(1f, 0f, 0f, 0f) // [w,x,y,z] EKFからの出力
    private var currentRotationMatrix: FloatArray = FloatArray(9).apply { this[0]=1f; this[4]=1f; this[8]=1f } // EKFクォータニオンから計算
    private var initialAttitudeQuaternion: FloatArray? = null // 相対角度計算用
    private var isAttitudeInitialized = false // AttitudeEKFが最初の観測を得たか
    private var gyroLastTimestamp_att: Long = 0L // 姿勢EKF予測用
    private val NS2S = 1.0f / 1000_000_000.0f

    //定数
    companion object {
        //(位置EKF, GPS等の定数は変更なし)
        private const val ACCUMULATION_INTERVAL_SECONDS = 1.0f
        private const val ACCUMULATION_INTERVAL_MS = (ACCUMULATION_INTERVAL_SECONDS * 1000).toLong()
        private const val MIN_ACCUMULATION_DISTANCE_THRESHOLD = 0.05f
        private const val MAX_ACCUMULATION_STEP_DISTANCE = 10.0f
        private const val stationaryAccelThreshold = 0.08f
        private const val stationaryGyroThreshold = 0.03f
        private const val MIN_VELOCITY_THRESHOLD = 0.1f
        private const val GPS_CONSISTENCY_DISTANCE_THRESHOLD = 2.0f
        private const val GPS_STATIONARY_DISTANCE_RATIO = 0.75f
        private const val HIGH_SPEED_THRESHOLD_KMH = 5.0f
        private const val HIGH_SPEED_THRESHOLD_MPS = HIGH_SPEED_THRESHOLD_KMH / 3.6f
        private val MIN_DISTANCE_TO_ADD_HISTORY = 0.1f
        //ジンバルロック検出は AttitudeEKF が行わないため不要に
    }

    //StateFlow
    private val _motionStateFlow = MutableStateFlow(MotionState.initial())
    val motionStateFlow: StateFlow<MotionState> = _motionStateFlow.asStateFlow()


    fun reset() {
        Log.i("MotionProcessor", "Resetting MotionProcessor state...")
        positionEKF.reset() // 位置EKFリセット
        attitudeEKF.reset() // ★ 姿勢EKFリセット

        //位置関連リセット
        worldAccel.fill(0f)
        accelLastTimestamp = 0L
        isAccelFirstTime = true
        currentGyroRate.fill(0f)
        isStationary = false
        isHighSpeedMode = false
        latestGpsLocationData = null
        lastValidGpsData = null
        lastEkfPositionAtGpsUpdate = null
        positionHistory.clear()
        lastPositionAddedToHistory = null
        accumulatedDistance = 0f
        lastDistanceAccumulationTime = 0L
        lastPositionAtAccumulation = null

        //姿勢関連リセット
        currentAttitudeQuaternion = floatArrayOf(1f, 0f, 0f, 0f)
        setIdentityMatrix(currentRotationMatrix)
        initialAttitudeQuaternion = null
        isAttitudeInitialized = false
        gyroLastTimestamp_att = 0L

        _motionStateFlow.value = MotionState.initial()
        Log.i("MotionProcessor", "Processor reset complete.")
    }

    /*
      ジャイロスコープイベント処理
      姿勢EKFの予測ステップを実行
      静止判定用の角速度を保持
     */
    fun processGyroscopeEvent(event: SensorEvent) {
        val currentTimestamp = event.timestamp
        val gx = event.values[0] //rad/s
        val gy = event.values[1]
        val gz = event.values[2]

        //静止判定用
        currentGyroRate[0] = gx
        currentGyroRate[1] = gy
        currentGyroRate[2] = gz

        //姿勢EKF 予測
        if (gyroLastTimestamp_att > 0) {
            val dt = (currentTimestamp - gyroLastTimestamp_att) * NS2S
            if (dt > 1e-7f) {
                attitudeEKF.predict(currentGyroRate, dt)
                //予測後のクォータニオンを取得しておく（updateがない場合も最新を使うため）
                currentAttitudeQuaternion = attitudeEKF.getQuaternion()
                //必要なら回転行列も更新
                //AttitudeEKF.getRotationMatrixFromQuaternion(currentAttitudeQuaternion, currentRotationMatrix)
            }
        } else if (isAttitudeInitialized) {
            //初期化後最初のジャイロイベントならタイムスタンプだけ記録
            gyroLastTimestamp_att = currentTimestamp
        }
        //常にタイムスタンプを更新 (次の dt 計算用) - predict の中に入れるべきか？
        //いや、dt計算後に更新するのが正しい
        if (gyroLastTimestamp_att > 0 && (currentTimestamp - gyroLastTimestamp_att) * NS2S > 1e-7f) {
            gyroLastTimestamp_att = currentTimestamp
        } else if (isAttitudeInitialized && gyroLastTimestamp_att == 0L) {
            gyroLastTimestamp_att = currentTimestamp // 初期化後初回
        }

        //ジャイロだけのイベントでは updateMotionState は呼ばない（更新頻度が高すぎるため）
    }

    /*
      回転ベクトルイベント処理
      姿勢EKFの更新ステップを実行
      初期姿勢を設定
     */
    fun processRotationVectorEvent(event: SensorEvent) {
        if (event.values.size >= 4) {
            //センサーから観測クォータニオンを取得
            val measurementQuaternion = FloatArray(4)
            //SensorManagerの関数を使うのが安定していることが多い
            try {
                //getQuaternionFromVectorは正規化しないことがあるので注意
                SensorManager.getQuaternionFromVector(measurementQuaternion, event.values)
                //手動で正規化 [w, x, y, z] の順序に注意！ getQuaternionFromVector は [x, y, z, w] で返すことがある
                //Androidドキュメント: values[3] = cos(theta/2), values[0..2] = axis * sin(theta/2)
                //w = values[3], x = values[0], y = values[1], z = values[2]
                val w = measurementQuaternion[3]
                val x = measurementQuaternion[0]
                val y = measurementQuaternion[1]
                val z = measurementQuaternion[2]
                val sensorQuatWXYZ = floatArrayOf(w, x, y, z) // EKF内部と順序を合わせる

                //正規化
                val norm = sqrt(w*w + x*x + y*y + z*z)
                if (norm > 1e-9f) {
                    val invNorm = 1.0f / norm
                    sensorQuatWXYZ[0] *= invNorm
                    sensorQuatWXYZ[1] *= invNorm
                    sensorQuatWXYZ[2] *= invNorm
                    sensorQuatWXYZ[3] *= invNorm

                    //姿勢EKF 更新
                    attitudeEKF.update(sensorQuatWXYZ)

                    //EKFから最新の推定クォータニオンを取得
                    currentAttitudeQuaternion = attitudeEKF.getQuaternion()

                    //対応する回転行列を計算
                    AttitudeEKF.getRotationMatrixFromQuaternion(currentAttitudeQuaternion, currentRotationMatrix)

                    //初期姿勢の設定 (初回のみ)
                    if (!isAttitudeInitialized) {
                        initialAttitudeQuaternion = currentAttitudeQuaternion.clone()
                        isAttitudeInitialized = true
                        gyroLastTimestamp_att = event.timestamp //ジャイロ予測を開始
                        Log.i("MotionProcessor", "Attitude EKF initialized.")
                        //初期化直後のUI更新
                        updateMotionState()
                    }

                } else {
                    Log.w("MotionProcessor", "Rotation vector norm is too small.")
                }

            } catch (e: IllegalArgumentException) {
                Log.e("MotionProcessor", "Error getting quaternion from rotation vector: ${e.message}")
            }
        } else {
            Log.w("MotionProcessor", "Received rotation vector event with unexpected size: ${event.values.size}")
        }

        //updateMotionState()
    }

    /*
      線形加速度イベント処理
      位置EKFの予測ステップを実行 (姿勢EKFの回転行列を使用)
      EKF状態, GPS等に基づいてUI状態を更新
     */
    fun processLinearAccelerationEvent(event: SensorEvent) {
        //姿勢EKFが初期化されるまで待つ
        if (!isAttitudeInitialized) {
            return
        }

        val currentTimestamp = event.timestamp
        //(加速度タイムスタンプ、dt 計算 - 変更なし)
        if (isAccelFirstTime || accelLastTimestamp == 0L) {
            accelLastTimestamp = currentTimestamp; isAccelFirstTime = false; return
        }
        val dt = (currentTimestamp - accelLastTimestamp) * NS2S
        if (dt <= 1e-7f) { accelLastTimestamp = currentTimestamp; return }
        accelLastTimestamp = currentTimestamp

        //デバイス加速度
        val devAccX = event.values[0]; val devAccY = event.values[1]; val devAccZ = event.values[2]

        //ワールド加速度計算 (姿勢EKFの回転行列を使用)
        //currentRotationMatrix は processRotationVectorEvent で更新されているはず
        worldAccel[0] = currentRotationMatrix[0] * devAccX + currentRotationMatrix[1] * devAccY + currentRotationMatrix[2] * devAccZ
        worldAccel[1] = currentRotationMatrix[3] * devAccX + currentRotationMatrix[4] * devAccY + currentRotationMatrix[5] * devAccZ
        worldAccel[2] = currentRotationMatrix[6] * devAccX + currentRotationMatrix[7] * devAccY + currentRotationMatrix[8] * devAccZ

        //位置EKF 処理 (変更なし)
        //(静止判定)
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
            if (!isStationary) isStationary = true
            positionEKF.updateStationary()
            positionEKF.predict(dt, floatArrayOf(0f, 0f, 0f))
        } else {
            if (isStationary) isStationary = false
            positionEKF.predict(dt, worldAccel) //ワールド加速度を入力
        }

        //(EKF更新前の位置保持)
        val ekfPositionBeforeGpsUpdate = positionEKF.getPosition().clone()

        //(GPSデータによる位置EKF更新)
        val gpsDataToUse = latestGpsLocationData
        if (gpsDataToUse != null && gpsDataToUse.isLocalValid) {
            val updateSuccessful = try {
                positionEKF.updateGpsPosition(gpsDataToUse, isHighSpeedMode)
                true
            } catch (e: Exception) { false }

            if (updateSuccessful) {
                //(矛盾補正処理 - 略)
                lastValidGpsData = gpsDataToUse.copy()
                lastEkfPositionAtGpsUpdate = ekfPositionBeforeGpsUpdate
            }
        }

        //(低速度クリッピング)
        val currentVelocity = positionEKF.getVelocity()
        val currentSpeed = sqrt(currentVelocity[0].pow(2) + currentVelocity[1].pow(2) + currentVelocity[2].pow(2))
        if (!isStationary && currentSpeed < MIN_VELOCITY_THRESHOLD) {
            if (positionEKF.x[3, 0] != 0f || positionEKF.x[4, 0] != 0f || positionEKF.x[5, 0] != 0f) {
                positionEKF.x[3, 0] = 0f; positionEKF.x[4, 0] = 0f; positionEKF.x[5, 0] = 0f
            }
        }
        //位置EKF 処理 ここまで

        //距離・軌跡計算 (変更なし)
        val currentPositionArray = positionEKF.getPosition()
        val currentPointF = PointF(currentPositionArray[0], currentPositionArray[1])
        //(移動総距離の計算 - 略)
        //(軌跡履歴への追加 - 略)


        //最後にUI状態を更新
        updateMotionState()
    }

    /**
     * GPSデータ処理 (高速モード判定)
     * (変更なし)
     */
    fun processGpsData(locationData: LocationData?) {
        //(元のコードと同じ)
        this.latestGpsLocationData = locationData
        if (locationData != null) {
            val currentGpsSpeed = locationData.speed
            val isValidAndAccurate = locationData.isLocalValid && (locationData.accuracy ?: Float.MAX_VALUE) < 30f
            if (currentGpsSpeed != null && isValidAndAccurate) {
                val enteringHighSpeed = currentGpsSpeed > HIGH_SPEED_THRESHOLD_MPS && !isHighSpeedMode
                val exitingHighSpeed = currentGpsSpeed <= HIGH_SPEED_THRESHOLD_MPS && isHighSpeedMode
                if (enteringHighSpeed) isHighSpeedMode = true
                else if (exitingHighSpeed) isHighSpeedMode = false
            } else {
                if (isHighSpeedMode) isHighSpeedMode = false
            }
        } else {
            if (isHighSpeedMode) isHighSpeedMode = false
        }
    }

    /*
      現在の状態に基づいて MotionState を更新し、UIに通知
      姿勢EKFの結果から相対角度を計算して使用
     */
    private fun updateMotionState() {
        //姿勢EKFが初期化されるまで更新しない
        if (!isAttitudeInitialized || initialAttitudeQuaternion == null) {
            //初期化前は初期状態を流し続ける
            if (_motionStateFlow.value != MotionState.initial()) {
                _motionStateFlow.value = MotionState.initial()
            }
            return
        }

        //位置・速度情報の取得 (変更なし)
        val posState = positionEKF.getState()
        val position = floatArrayOf(posState[0], posState[1], posState[2])
        val velocity = floatArrayOf(posState[3], posState[4], posState[5])
        val speedMps = sqrt(velocity[0].pow(2) + velocity[1].pow(2) + velocity[2].pow(2))
        val speedKmh = speedMps * 3.6f
        val totalDistance = sqrt(position[0].pow(2) + position[1].pow(2) + position[2].pow(2))

        //相対角度の計算
        //Q_relative = Q_current * Q_initial_inverse
        val initialInverse = inverseQuaternion(initialAttitudeQuaternion!!)
        val relativeQuaternion = multiplyQuaternions(currentAttitudeQuaternion, initialInverse)
        normalizeQuaternion(relativeQuaternion) //正規化

        //相対クォータニオンから相対回転行列を計算
        val relativeRotationMatrix = FloatArray(9)
        AttitudeEKF.getRotationMatrixFromQuaternion(relativeQuaternion, relativeRotationMatrix)

        //相対回転行列から相対オイラー角 [Yaw, Pitch, Roll] (ラジアン) を計算
        val relativeOrientationRad = FloatArray(3)
        AttitudeEKF.getOrientationFromRotationMatrix(relativeRotationMatrix, relativeOrientationRad)

        /*
        //度数法に変換し、正規化
        val finalYaw = normalizeAngle(Math.toDegrees(relativeOrientationRad[0].toDouble()).toFloat())
        val finalPitch = normalizeAngle(Math.toDegrees(relativeOrientationRad[1].toDouble()).toFloat())
        val finalRoll = normalizeAngle(Math.toDegrees(relativeOrientationRad[2].toDouble()).toFloat())
        //相対角度計算ここまで
        */


        // 度数法に変換
        val rawYawDeg = Math.toDegrees(relativeOrientationRad[0].toDouble()).toFloat()
        val rawPitchDeg = Math.toDegrees(relativeOrientationRad[1].toDouble()).toFloat()
        val rawRollDeg = Math.toDegrees(relativeOrientationRad[2].toDouble()).toFloat()

        // 正規化
        val finalYaw = normalizeAngle(rawYawDeg)
        val finalPitch = normalizeAngle(rawPitchDeg)
        val finalRoll = normalizeAngle(rawRollDeg)

        //状態表示テキスト (ジンバルロックの項目は不要に)
        val statusText: String
        val statusColor: Int
        when {
            isStationary -> { statusText = "状態: 静止"; statusColor = android.R.color.holo_green_dark }
            isHighSpeedMode -> { statusText = "状態: 高速移動"; statusColor = android.R.color.holo_red_light }
            speedMps < MIN_VELOCITY_THRESHOLD && !isStationary -> { statusText = "状態: 低速"; statusColor = android.R.color.holo_blue_dark }
            else -> { statusText = "状態: 移動中"; statusColor = android.R.color.holo_orange_dark }
        }

        //MotionState 作成
        val newState = MotionState(
            position = position,
            velocity = velocity,
            worldAccel = worldAccel.clone(),
            angleYaw = finalYaw,
            anglePitch = finalPitch,
            angleRoll = finalRoll,
            isStationary = this.isStationary,
            isHighSpeedMode = this.isHighSpeedMode,
            speedMps = speedMps,
            speedKmh = speedKmh,
            totalDistance = totalDistance,
            accumulatedDistance = this.accumulatedDistance,
            currentPoint = PointF(position[0], position[1]),
            pathHistory = positionHistory.toList(),
            statusText = statusText,
            statusColor = statusColor,
            latestGpsData = this.latestGpsLocationData?.copy()
        )

        //StateFlow 更新
        if (_motionStateFlow.value != newState) {
            _motionStateFlow.value = newState
        }
    }

    private fun normalizeAngle(angle: Float): Float {
        var normalized = angle % 360.0f
        if (normalized > 180.0f) {
            normalized -= 360.0f
        } else if (normalized <= -180.0f) {
            normalized += 360.0f
        }
        return normalized
    }

    private fun setIdentityMatrix(matrix: FloatArray) {
        if (matrix.size < 9) return
        matrix.fill(0f)
        matrix[0] = 1f; matrix[4] = 1f; matrix[8] = 1f
    }

    //クォータニオン演算ヘルパー (AttitudeEKFに static で置くか、ここに置くか)
    private fun inverseQuaternion(q: FloatArray): FloatArray {
        //Assuming normalized: q^-1 = [w, -x, -y, -z] (if q=[w,x,y,z])
        return floatArrayOf(q[0], -q[1], -q[2], -q[3])
    }

    private fun multiplyQuaternions(q1: FloatArray, q2: FloatArray): FloatArray {
        //q_result = q1 * q2 ; q = [w, x, y, z]
        val w1 = q1[0]; val x1 = q1[1]; val y1 = q1[2]; val z1 = q1[3]
        val w2 = q2[0]; val x2 = q2[1]; val y2 = q2[2]; val z2 = q2[3]
        val w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        val x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        val y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        val z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return floatArrayOf(w, x, y, z)
    }

    private fun normalizeQuaternion(q: FloatArray) {
        val norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        if (norm > 1e-9f) {
            val invNorm = 1.0f / norm
            q[0] *= invNorm; q[1] *= invNorm; q[2] *= invNorm; q[3] *= invNorm
        } else {
            q[0] = 1.0f; q[1] = 0f; q[2] = 0f; q[3] = 0f // fallback to identity
        }
    }
}