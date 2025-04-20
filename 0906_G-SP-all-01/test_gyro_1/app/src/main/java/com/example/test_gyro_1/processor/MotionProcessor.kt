// 例: com/example/test_gyro_1/processor/MotionProcessor.kt
package com.example.test_gyro_1.processor

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
import kotlin.math.sqrt

class MotionProcessor {

    //EKF関連
    private val ekf = ExtendedKalmanFilter()

    //状態変数
    private val worldAccel = FloatArray(3)
    private val rotationMatrix = FloatArray(9).apply { this[0] = 1f; this[4] = 1f; this[8] = 1f } // 単位行列で初期化
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

    //移動総距離計算用
    private var accumulatedDistance: Float = 0f //計算された総移動距離
    private var lastDistanceAccumulationTime: Long = 0L //最後に距離を加算した時刻 (ms)
    private var lastPositionAtAccumulation: FloatArray? = null //最後に距離を加算した時のEKF位置

    //設定可能な定数
    companion object { //距離を加算する時間間隔 (秒単位で設定)
        private const val ACCUMULATION_INTERVAL_SECONDS = 5.0f //vf1.0
        //ミリ秒に変換
        private const val ACCUMULATION_INTERVAL_MS = (ACCUMULATION_INTERVAL_SECONDS * 1000).toLong()
        private const val MIN_ACCUMULATION_DISTANCE_THRESHOLD = 0.50f //bf0.05
        private const val MAX_ACCUMULATION_STEP_DISTANCE = 10.0f //例:10m
    }

    //定数・閾値
    private val NS2S = 1.0f / 1000000000.0f
    private val driftThreshold = 0.01f
    private val stationaryAccelThreshold = 0.05f
    private val stationaryGyroThreshold = 0.05f
    private val MIN_VELOCITY_THRESHOLD = 0.1f
    private val GPS_CONSISTENCY_DISTANCE_THRESHOLD = 3.0f
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
        lastDistanceAccumulationTime = 0L //0Lにリセット(最初の更新で時刻が設定される)
        lastPositionAtAccumulation = null

        _motionStateFlow.value = MotionState.initial() //StateFlowも初期化
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


        //EKF更新前の位置を保持(GPS矛盾チェック用)
        val ekfPositionBeforeGpsUpdate = ekf.getPosition().clone()

        //GPSデータによるEKF更新
        val gpsDataToUse = latestGpsLocationData
        //var gpsUpdated = false //gpsUpdated は使われていないので削除してもOK
        if (gpsDataToUse != null && gpsDataToUse.isLocalValid) {
            //EKFクラス内でタイムスタンプ比較や短距離スキップが行われる
            Log.d("MotionProcessor", "Attempting EKF update with GPS data (Timestamp: ${gpsDataToUse.timestamp}, HighSpeed: $isHighSpeedMode)")
            val updateSuccessful = try {
                ekf.updateGpsPosition(gpsDataToUse, isHighSpeedMode)
                true //updateGpsPositionが例外を投げなければ成功とみなす（戻り値がないため）
            } catch (e: Exception) {
                Log.e("MotionProcessor", "Error during EKF GPS update", e)
                false
            }

            if (updateSuccessful) {
                //gpsUpdated = true

                //IMU/GPS矛盾検出と補正
                val lastGps = lastValidGpsData //最後に矛盾チェックに使ったGPS
                val lastEkfPos = lastEkfPositionAtGpsUpdate //その時のEKF位置
                if (lastGps != null && lastGps.isLocalValid && lastEkfPos != null &&
                    gpsDataToUse.localX != null && gpsDataToUse.localY != null &&
                    lastGps.localX != null && lastGps.localY != null) {

                    val dxGps = gpsDataToUse.localX - lastGps.localX
                    val dyGps = gpsDataToUse.localY - lastGps.localY
                    val distGps = sqrt(dxGps * dxGps + dyGps * dyGps)
                    val dxEkf = ekfPositionBeforeGpsUpdate[0] - lastEkfPos[0] //更新"前"のEKF位置と比較
                    val dyEkf = ekfPositionBeforeGpsUpdate[1] - lastEkfPos[1]
                    val distEkf = sqrt(dxEkf * dxEkf + dyEkf * dyEkf)
                    val gpsAccuracy = gpsDataToUse.accuracy ?: 30.0f
                    val gpsStationaryThreshold = gpsAccuracy * GPS_STATIONARY_DISTANCE_RATIO

                    if (distEkf > GPS_CONSISTENCY_DISTANCE_THRESHOLD && distGps < gpsStationaryThreshold) {
                        Log.w("MotionProcessor", "Inconsistency! EKF moved %.2fm, GPS moved %.2fm (threshold %.2fm). Correcting EKF position.".format(distEkf, distGps, gpsStationaryThreshold))

                        //EKFの位置を補正 (EKF更新後の値を使う)
                        val correctedEkfPosition = ekf.getPosition() //更新後のEKF位置を取得
                        correctedEkfPosition[0] = lastEkfPos[0] + (correctedEkfPosition[0] - lastEkfPos[0]) * 0.5f
                        correctedEkfPosition[1] = lastEkfPos[1] + (correctedEkfPosition[1] - lastEkfPos[1]) * 0.5f
                        if (correctedEkfPosition.size > 2 && lastEkfPos.size > 2 && gpsDataToUse.localZ != null && lastGps.localZ != null) {
                            //Z座標も同様に補正する場合
                        }

                        //EKFの状態ベクトルに直接書き込む
                        ekf.x[0, 0] = correctedEkfPosition[0]
                        ekf.x[1, 0] = correctedEkfPosition[1]
                        if (correctedEkfPosition.size > 2) {
                            ekf.x[2, 0] = correctedEkfPosition[2]
                        }
                        Log.w("MotionProcessor", "Corrected EKF pos: X=%.2f, Y=%.2f, Z=%.2f".format(ekf.x[0, 0], ekf.x[1, 0], ekf.x[2, 0]))
                    }
                }
                //今回の矛盾チェックに使用したGPSデータと、その直前(GPS更新前)のEKF位置を保存
                lastValidGpsData = gpsDataToUse.copy()
                lastEkfPositionAtGpsUpdate = ekfPositionBeforeGpsUpdate //更新"前"の位置を保存
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


        //移動総距離の計算
        val currentPosition = ekf.getPosition() //EKF更新後の最新位置を取得
        val currentTime = System.currentTimeMillis()

        //最後に記録した位置があるか？
        val lastPos = lastPositionAtAccumulation
        if (lastPos != null) {
            //最後に距離を加算してから指定時間が経過したか？
            if (currentTime - lastDistanceAccumulationTime >= ACCUMULATION_INTERVAL_MS) {
                val dx = currentPosition[0] - lastPos[0]
                val dy = currentPosition[1] - lastPos[1]
                //Z座標も考慮する場合 (XYZの3次元距離)
                val dz = currentPosition[2] - lastPos[2]
                val deltaDistance = sqrt(dx * dx + dy * dy + dz * dz)

                //ノイズ除去とジャンプ対策
                if (deltaDistance >= MIN_ACCUMULATION_DISTANCE_THRESHOLD && deltaDistance <= MAX_ACCUMULATION_STEP_DISTANCE) {
                    accumulatedDistance += deltaDistance
                    Log.d("MotionProcessor", "Accumulated distance: +%.2fm (Total: %.2fm)".format(deltaDistance, accumulatedDistance))
                } else if (deltaDistance > MAX_ACCUMULATION_STEP_DISTANCE) {
                    Log.w("MotionProcessor", "Skipping large distance step: %.2f m > %.1f m".format(deltaDistance, MAX_ACCUMULATION_STEP_DISTANCE))
                }

                //時刻と位置を更新
                lastDistanceAccumulationTime = currentTime
                lastPositionAtAccumulation = currentPosition.clone()
            }
        } else {
            //最初の位置を記録
            lastPositionAtAccumulation = currentPosition.clone()
            lastDistanceAccumulationTime = currentTime
        }

        updateMotionState()
    }

    //GPSデータ処理
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


    //現在の内部状態からMotionStateオブジェクトを生成し、StateFlowを更新
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
                statusColor = R.color.holo_blue_dark // android.R を想定
            }
            else -> {
                statusText = "状態: 移動中 (EKF予測中)"
                statusColor = R.color.holo_orange_dark //android.R を想定
            }
        }

        val newState = MotionState(
            position = position,
            velocity = velocity,
            worldAccel = worldAccel.clone(), // cloneして渡す
            angleX = this.angleX,
            angleY = this.angleY,
            angleZ = this.angleZ,
            isStationary = this.isStationary,
            isHighSpeedMode = this.isHighSpeedMode,
            speedMps = speedMps,
            speedKmh = speedKmh,
            totalDistance = totalDistance,
            accumulatedDistance = this.accumulatedDistance,
            statusText = statusText,
            statusColor = statusColor,
            latestGpsData = this.latestGpsLocationData?.copy()
        )

        //現在の値と異なれば更新 (無駄な更新を避ける)
        if (_motionStateFlow.value != newState) {
            _motionStateFlow.value = newState
        }
    }

    //角度正規化
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