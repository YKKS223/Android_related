package com.example.test_gyro_1.processor

import android.graphics.PointF // ★ Import
import android.hardware.SensorEvent
import android.hardware.SensorManager
import android.R
import android.util.Log
import com.example.test_gyro_1.filter.ExtendedKalmanFilter
import com.example.test_gyro_1.gps.LocationData
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs
import kotlin.math.pow // ★ pow をインポート
import kotlin.math.sqrt

class MotionProcessor {

    //EKF関連
    private val ekf = ExtendedKalmanFilter()

    //状態変数
    private val worldAccel = FloatArray(3)
    private val rotationMatrix = FloatArray(9).apply { this[0] = 1f; this[4] = 1f; this[8] = 1f } //単位行列で初期化
    private var hasRotationVector = false
    private var angleX = 0f
    private var angleY = 0f
    private var angleZ = 0f
    private var gyroLastTimestamp: Long = 0
    private var accelLastTimestamp: Long = 0
    private var isGyroFirstTime = true
    private var isAccelFirstTime = true
    private var currentGyroRate = FloatArray(3)
    private var isStationary = false
    private var isHighSpeedMode = false
    private var latestGpsLocationData: LocationData? = null //保持する最新のGPSデータ
    private var lastValidGpsData: LocationData? = null //矛盾チェック用
    private var lastEkfPositionAtGpsUpdate: FloatArray? = null //矛盾チェック用

    //軌跡履歴用
    private val positionHistory = mutableListOf<PointF>()
    private var lastPositionAddedToHistory: PointF? = null //履歴に追加した最後の位置(PointF型)
    private val MIN_DISTANCE_TO_ADD_HISTORY = 0.5f //bf0.1

    //移動総距離計算用
    private var accumulatedDistance: Float = 0f //計算された総移動距離
    private var lastDistanceAccumulationTime: Long = 0L //最後に距離を加算した時刻(ms)
    private var lastPositionAtAccumulation: FloatArray? = null //最後に距離を加算した時のEKF位置(FloatArray型)

    //設定可能な定数
    companion object { //距離を加算する時間間隔 (秒単位で設定)
        private const val ACCUMULATION_INTERVAL_SECONDS = 5.0f //bf1.0
        //ミリ秒に変換
        private const val ACCUMULATION_INTERVAL_MS = (ACCUMULATION_INTERVAL_SECONDS * 1000).toLong()
        //閾値はこちらで定義
        private const val MIN_ACCUMULATION_DISTANCE_THRESHOLD = 0.50f //bf0.05
        private const val MAX_ACCUMULATION_STEP_DISTANCE = 10.0f //例: 10m
    }

    //定数・閾値
    private val NS2S = 1.0f / 1000000000.0f
    private val driftThreshold = 0.01f
    private val stationaryAccelThreshold = 0.05f
    private val stationaryGyroThreshold = 0.05f
    private val MIN_VELOCITY_THRESHOLD = 0.1f
    private val GPS_CONSISTENCY_DISTANCE_THRESHOLD = 2.0f
    private val GPS_STATIONARY_DISTANCE_RATIO = 0.75f
    private val HIGH_SPEED_THRESHOLD_KMH = 5.0f
    private val HIGH_SPEED_THRESHOLD_MPS = HIGH_SPEED_THRESHOLD_KMH / 3.6f

    //UIへ状態を通知するための StateFlow
    private val _motionStateFlow = MutableStateFlow(MotionState.initial())
    val motionStateFlow: StateFlow<MotionState> = _motionStateFlow.asStateFlow()

    //メソッド

    fun reset() {
        ekf.reset()
        worldAccel.fill(0f)
        rotationMatrix[0] = 1f; rotationMatrix[4] = 1f; rotationMatrix[8] = 1f; //単位行列
        hasRotationVector = false
        angleX = 0f; angleY = 0f; angleZ = 0f
        gyroLastTimestamp = 0L
        accelLastTimestamp = 0L
        isGyroFirstTime = true
        isAccelFirstTime = true
        currentGyroRate.fill(0f)
        isStationary = false
        isHighSpeedMode = false
        latestGpsLocationData = null
        lastValidGpsData = null
        lastEkfPositionAtGpsUpdate = null
        //総距離関連もリセット
        accumulatedDistance = 0f
        lastDistanceAccumulationTime = 0L
        lastPositionAtAccumulation = null
        //軌跡履歴もリセット
        positionHistory.clear()
        lastPositionAddedToHistory = null

        _motionStateFlow.value = MotionState.initial()
        Log.d("MotionProcessor", "Processor reset.")
    }

    fun processGyroscopeEvent(event: SensorEvent) {
        currentGyroRate[0] = event.values[0]
        currentGyroRate[1] = event.values[1]
        currentGyroRate[2] = event.values[2]

        if (isGyroFirstTime) {
            gyroLastTimestamp = event.timestamp
            isGyroFirstTime = false
            return
        }
        if (gyroLastTimestamp == 0L) {
            gyroLastTimestamp = event.timestamp
            return
        }

        val dt = (event.timestamp - gyroLastTimestamp) * NS2S
        if (dt <= 0.000001f) {
            gyroLastTimestamp = event.timestamp
            return
        }
        gyroLastTimestamp = event.timestamp

        var axisX = event.values[0]
        var axisY = event.values[1]
        var axisZ = event.values[2]

        if (abs(axisX) < driftThreshold) axisX = 0f
        if (abs(axisY) < driftThreshold) axisY = 0f
        if (abs(axisZ) < driftThreshold) axisZ = 0f

        angleX += Math.toDegrees((axisX * dt).toDouble()).toFloat()
        angleY += Math.toDegrees((axisY * dt).toDouble()).toFloat()
        angleZ += Math.toDegrees((axisZ * dt).toDouble()).toFloat()

        angleX = normalizeAngle(angleX)
        angleY = normalizeAngle(angleY)
        angleZ = normalizeAngle(angleZ)
    }

    fun processRotationVectorEvent(event: SensorEvent) {
        try {
            SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
            hasRotationVector = true
        } catch (e: IllegalArgumentException) {
            Log.e("MotionProcessor", "IllegalArgumentException from getRotationMatrixFromVector: ${e.message}")
            hasRotationVector = false
        } catch (e: Exception) {
            Log.e("MotionProcessor", "Error processing Rotation Vector event: ${e.message}", e)
            hasRotationVector = false
        }
    }


    fun processLinearAccelerationEvent(event: SensorEvent) {
        if (!hasRotationVector) return

        if (isAccelFirstTime) {
            accelLastTimestamp = event.timestamp
            isAccelFirstTime = false
            return
        }
        if (accelLastTimestamp == 0L) {
            accelLastTimestamp = event.timestamp
            return
        }
        val dt = (event.timestamp - accelLastTimestamp) * NS2S
        if (dt <= 0.000001f) {
            accelLastTimestamp = event.timestamp
            return
        }
        accelLastTimestamp = event.timestamp

        //ワールド加速度計算
        val deviceAccelX = event.values[0]
        val deviceAccelY = event.values[1]
        val deviceAccelZ = event.values[2]
        worldAccel[0] = rotationMatrix[0] * deviceAccelX + rotationMatrix[1] * deviceAccelY + rotationMatrix[2] * deviceAccelZ
        worldAccel[1] = rotationMatrix[3] * deviceAccelX + rotationMatrix[4] * deviceAccelY + rotationMatrix[5] * deviceAccelZ
        worldAccel[2] = rotationMatrix[6] * deviceAccelX + rotationMatrix[7] * deviceAccelY + rotationMatrix[8] * deviceAccelZ

        //静止判定
        val accelMagnitude = sqrt(worldAccel[0] * worldAccel[0] + worldAccel[1] * worldAccel[1] + worldAccel[2] * worldAccel[2])
        val gyroMagnitude = sqrt(currentGyroRate[0] * currentGyroRate[0] + currentGyroRate[1] * currentGyroRate[1] + currentGyroRate[2] * currentGyroRate[2])
        val currentGpsSpeedMps = latestGpsLocationData?.speed ?: 0.0f
        val isGpsSpeedBelowThreshold = currentGpsSpeedMps < HIGH_SPEED_THRESHOLD_MPS
        val isImuStationary = accelMagnitude < stationaryAccelThreshold && gyroMagnitude < stationaryGyroThreshold
        val isConsideredStationary = isImuStationary && isGpsSpeedBelowThreshold
        val justBecameStationary = !isStationary && isConsideredStationary
        val isImuNearlyStationary = accelMagnitude < stationaryAccelThreshold * 1.5f && gyroMagnitude < stationaryGyroThreshold * 1.5f
        val remainsStationary = isStationary && isImuNearlyStationary && isGpsSpeedBelowThreshold

        if (justBecameStationary || remainsStationary) {
            if (!isStationary) {
                Log.i("MotionProcessor", "Detected stationary state (IMU stationary AND GPS speed < %.1f km/h)".format(HIGH_SPEED_THRESHOLD_KMH))
                isStationary = true
            }
            ekf.updateStationary()
            ekf.predict(dt, floatArrayOf(0f, 0f, 0f))
        } else {
            if (isStationary) {
                Log.i("MotionProcessor", "Exited stationary state (Reason: IMU moving OR GPS speed >= %.1f km/h)".format(HIGH_SPEED_THRESHOLD_KMH))
                isStationary = false
            }
            ekf.predict(dt, worldAccel)
        }

        //EKF更新前の位置を保持 (GPS矛盾チェック用)
        val ekfPositionBeforeGpsUpdate = ekf.getPosition().clone()

        //GPSデータによるEKF更新
        val gpsDataToUse = latestGpsLocationData
        if (gpsDataToUse != null && gpsDataToUse.isLocalValid) {
            //(EKF更新処理)
            Log.d("MotionProcessor", "Attempting EKF update with GPS data (Timestamp: ${gpsDataToUse.timestamp}, HighSpeed: $isHighSpeedMode)")
            val updateSuccessful = try {
                ekf.updateGpsPosition(gpsDataToUse, isHighSpeedMode)
                true
            } catch (e: Exception) {
                Log.e("MotionProcessor", "Error during EKF GPS update", e)
                false
            }

            if (updateSuccessful) {
                //(矛盾補正処理)
                val lastGps = lastValidGpsData
                val lastEkfPos = lastEkfPositionAtGpsUpdate
                if (lastGps != null && lastGps.isLocalValid && lastEkfPos != null &&
                    gpsDataToUse.localX != null && gpsDataToUse.localY != null &&
                    lastGps.localX != null && lastGps.localY != null) {

                    val dxGps = gpsDataToUse.localX - lastGps.localX
                    val dyGps = gpsDataToUse.localY - lastGps.localY
                    val distGps = sqrt(dxGps * dxGps + dyGps * dyGps)
                    val dxEkf = ekfPositionBeforeGpsUpdate[0] - lastEkfPos[0]
                    val dyEkf = ekfPositionBeforeGpsUpdate[1] - lastEkfPos[1]
                    val distEkf = sqrt(dxEkf * dxEkf + dyEkf * dyEkf)
                    val gpsAccuracy = gpsDataToUse.accuracy ?: 30.0f
                    val gpsStationaryThreshold = gpsAccuracy * GPS_STATIONARY_DISTANCE_RATIO

                    if (distEkf > GPS_CONSISTENCY_DISTANCE_THRESHOLD) {
                        Log.w("MotionProcessor", "Inconsistency! EKF moved %.2fm, GPS moved %.2fm (threshold %.2fm). Correcting EKF position.".format(distEkf, distGps, gpsStationaryThreshold))
                        val correctedEkfPosition = ekf.getPosition()
                        correctedEkfPosition[0] = lastEkfPos[0] + (correctedEkfPosition[0] - lastEkfPos[0]) * 0.4f
                        correctedEkfPosition[1] = lastEkfPos[1] + (correctedEkfPosition[1] - lastEkfPos[1]) * 0.4f
                        //Z補正
                        ekf.x[0, 0] = correctedEkfPosition[0]
                        ekf.x[1, 0] = correctedEkfPosition[1]
                        //Z書き込み
                        Log.w("MotionProcessor", "Corrected EKF pos: X=%.2f, Y=%.2f, Z=%.2f".format(ekf.x[0, 0], ekf.x[1, 0], ekf.x[2, 0]))
                    }
                }
                lastValidGpsData = gpsDataToUse.copy()
                lastEkfPositionAtGpsUpdate = ekfPositionBeforeGpsUpdate
            }
        }

        //低速度域での速度クリッピング
        val currentVelocity = ekf.getVelocity()
        val currentSpeed = sqrt(currentVelocity[0] * currentVelocity[0] +
                currentVelocity[1] * currentVelocity[1] +
                currentVelocity[2] * currentVelocity[2])
        if (currentSpeed < MIN_VELOCITY_THRESHOLD) {
            if (ekf.x[3, 0] != 0f || ekf.x[4, 0] != 0f || ekf.x[5, 0] != 0f) {
                Log.d("MotionProcessor", "Clipping speed below threshold (${MIN_VELOCITY_THRESHOLD} m/s). Speed: %.4f m/s".format(currentSpeed))
                ekf.x[3, 0] = 0f; ekf.x[4, 0] = 0f; ekf.x[5, 0] = 0f
            }
        }

        //EKF更新後の最新位置を取得
        val currentPositionArray = ekf.getPosition() //FloatArray [x, y, z]
        val currentPointF = PointF(currentPositionArray[0], currentPositionArray[1])

        //移動総距離の計算
        val currentTime = System.currentTimeMillis()
        val lastPosArray = lastPositionAtAccumulation //FloatArray or null
        if (lastPosArray != null) {
            if (currentTime - lastDistanceAccumulationTime >= ACCUMULATION_INTERVAL_MS) {
                val dx = currentPositionArray[0] - lastPosArray[0]
                val dy = currentPositionArray[1] - lastPosArray[1]
                val dz = currentPositionArray[2] - lastPosArray[2]
                val deltaDistance = sqrt(dx * dx + dy * dy + dz * dz)

                if (deltaDistance >= MIN_ACCUMULATION_DISTANCE_THRESHOLD && deltaDistance <= MAX_ACCUMULATION_STEP_DISTANCE) {
                    accumulatedDistance += deltaDistance
                    Log.d("MotionProcessor", "Accumulated distance: +%.2fm (Total: %.2fm)".format(deltaDistance, accumulatedDistance))
                } else if (deltaDistance > MAX_ACCUMULATION_STEP_DISTANCE) {
                    Log.w("MotionProcessor", "Skipping large distance step: %.2f m > %.1f m".format(deltaDistance, MAX_ACCUMULATION_STEP_DISTANCE))
                }

                lastDistanceAccumulationTime = currentTime
                lastPositionAtAccumulation = currentPositionArray.clone()
            }
        } else {
            lastPositionAtAccumulation = currentPositionArray.clone()
            lastDistanceAccumulationTime = currentTime
        }

        //軌跡履歴への追加
        val lastAddedPointF = lastPositionAddedToHistory //PointF or null
        var shouldAddToHistory = false
        if (lastAddedPointF == null) {
            shouldAddToHistory = true
        } else {
            //PointF を使って距離計算
            val distFromLast = sqrt((currentPointF.x - lastAddedPointF.x).pow(2) + (currentPointF.y - lastAddedPointF.y).pow(2))
            if (distFromLast >= MIN_DISTANCE_TO_ADD_HISTORY) {
                shouldAddToHistory = true
            }
        }

        if (shouldAddToHistory) {
            //PointF を履歴に追加
            positionHistory.add(currentPointF)
            lastPositionAddedToHistory = currentPointF // PointF を保持
            //オプション: 履歴のサイズ制限
            //if (positionHistory.size > 1000) {
            //positionHistory.removeAt(0)
            //}
        }

        //最後に状態を更新してUIに通知
        updateMotionState()
    }

    //(processGpsData は変更なし)
    fun processGpsData(locationData: LocationData?) {
        this.latestGpsLocationData = locationData

        if (locationData != null) {
            //高速移動モードの更新
            val currentGpsSpeed = locationData.speed
            val isValidAndAccurate = locationData.isLocalValid && (locationData.accuracy ?: Float.MAX_VALUE) < 30f

            if (currentGpsSpeed != null && isValidAndAccurate) {
                val enteringHighSpeed = currentGpsSpeed > HIGH_SPEED_THRESHOLD_MPS && !isHighSpeedMode
                val exitingHighSpeed = currentGpsSpeed <= HIGH_SPEED_THRESHOLD_MPS && isHighSpeedMode

                if (enteringHighSpeed) {
                    Log.i("MotionProcessor", "Entered High Speed Mode (GPS Speed: %.2f m/s)".format(currentGpsSpeed))
                    isHighSpeedMode = true
                } else if (exitingHighSpeed) {
                    Log.i("MotionProcessor", "Exited High Speed Mode (GPS Speed: %.2f m/s)".format(currentGpsSpeed))
                    isHighSpeedMode = false
                }
            } else {
                //GPS速度が無効または精度が悪ければ高速モード解除
                if (isHighSpeedMode) {
                    Log.i("MotionProcessor", "Exited High Speed Mode (Invalid GPS speed or accuracy)")
                    isHighSpeedMode = false
                }
            }
        } else {
            //GPSデータがない場合は高速モード解除
            if (isHighSpeedMode) {
                Log.i("MotionProcessor", "Exited High Speed Mode (GPS stopped)")
                isHighSpeedMode = false
            }
        }
        //GPSデータ更新時にも状態を通知(高速モードが変わった場合などに備える)
        updateMotionState()
    }


    //(updateMotionState は変更なし)
    private fun updateMotionState() {
        val currentState = ekf.getState()
        val position = floatArrayOf(currentState[0], currentState[1], currentState[2])
        val velocity = floatArrayOf(currentState[3], currentState[4], currentState[5])
        val speedMps = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2])
        val speedKmh = speedMps * 3.6f
        val totalDistance = sqrt(position[0] * position[0] + position[1] * position[1] + position[2] * position[2])

        //状態表示テキストと色を決定
        val statusText: String
        val statusColor: Int
        when {
            isStationary -> {
                statusText = "状態: 静止 (EKF更新中)"
                statusColor = R.color.holo_green_dark //android.R を想定
            }
            isHighSpeedMode -> {
                statusText = "状態: 高速移動 (GPS優先)"
                statusColor = R.color.holo_red_light //android.R を想定
            }
            speedMps < MIN_VELOCITY_THRESHOLD -> {
                statusText = "状態: 低速 (速度クリップ中)"
                statusColor = R.color.holo_blue_dark //android.R を想定
            }
            else -> {
                statusText = "状態: 移動中 (EKF予測中)"
                statusColor = R.color.holo_orange_dark //android.R を想定
            }
        }

        val newState = MotionState(
            position = position,
            velocity = velocity,
            worldAccel = worldAccel.clone(), //cloneして渡す
            angleX = this.angleX,
            angleY = this.angleY,
            angleZ = this.angleZ,
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

        //現在の値と異なれば更新 (無駄な更新を避ける)
        if (_motionStateFlow.value != newState) {
            _motionStateFlow.value = newState
        }
    }

    private fun normalizeAngle(angle: Float): Float {
        var normalizedAngle = angle % 360
        if (normalizedAngle > 180) {
            normalizedAngle -= 360
        } else if (normalizedAngle <= -180) {
            normalizedAngle += 360
        }
        return normalizedAngle
    }

}