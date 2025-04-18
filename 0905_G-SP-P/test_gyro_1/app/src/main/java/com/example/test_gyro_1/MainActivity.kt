// MainActivity.kt
package com.example.test_gyro_1

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log // デバッグ用に追加
import android.widget.TextView
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.max // max 関数を追加
import com.example.test_gyro_1.filter.ExtendedKalmanFilter
import com.example.test_gyro_1.filter.SimpleMatrix

import com.example.test_gyro_1.gps.GpsManager
import com.example.test_gyro_1.gps.LocationData
import kotlinx.coroutines.launch // コルーチン用
import androidx.lifecycle.lifecycleScope // コルーチン用

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var gyroscope: Sensor? = null //ジャイロセンサ
    private var linearAccelerometer: Sensor? = null //加速度センサ
    private var rotationVectorSensor: Sensor? = null //回転ベクトルセンサー

    private lateinit var textViewInfo: TextView //センサ情報表示用
    private lateinit var textViewX: TextView //角度表示用
    private lateinit var textViewY: TextView
    private lateinit var textViewZ: TextView
    private lateinit var textViewWarning: TextView //状態確認用
    private lateinit var textViewVelocity: TextView //速度(m/s)表示用
    private lateinit var textViewDistance: TextView //変位(m)表示用
    private lateinit var textViewWorldAcc: TextView //ワールド座標系の加速度表示用(デバッグ用)
    private lateinit var textViewVelocityh: TextView //時速(km/h)表示用

    //ジャイロセンサー関連
    private var angleX = 0f //ジャイロ積分角度 (EKF状態とは別)
    private var angleY = 0f
    private var angleZ = 0f
    private var gyroLastTimestamp: Long = 0
    private val NS2S = 1.0f / 1000000000.0f
    private val driftThreshold = 0.01f //bf0.01
    private var isGyroFirstTime = true
    private var currentGyroRate = FloatArray(3) //現在の角速度(rad/s)を保持

    // --- EKF関連 ---
    private lateinit var ekf: ExtendedKalmanFilter //EKFインスタンス
    private val worldAccel = FloatArray(3) //ワールド座標系の加速度(EKF入力用)

    //加速度センサーとセンサーフュージョン関連
    private var accelLastTimestamp: Long = 0
    private var isAccelFirstTime = true

    //静止状態検出のための閾値(調整が必要)
    private val stationaryAccelThreshold = 0.05f //m/s^2(この値以下の加速度の大きさは静止とみなす) bf0.1
    private val stationaryGyroThreshold = 0.05f //rad/s(この値以下の角速度の大きさは静止とみなす) bf0.02
    private val MIN_VELOCITY_THRESHOLD = 0.1f //m/s (この値以下の速度はゼロとみなす) bf0.1

    private var isStationary = false //静止状態フラグ

    //回転ベクトルセンサー関連
    private val rotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3) //回転角度 (デバッグ用)
    // private var currentRotationVector = FloatArray(4) //不要になった
    private var hasRotationVector = false

    //GPS関連
    private lateinit var gpsManager: GpsManager
    private var latestGpsLocationData: LocationData? = null //最新GPSデータ保持用
    //変位補正用
    private var lastValidGpsData: LocationData? = null       //最後にEKF更新に使った有効なGPSデータ
    private var lastEkfPositionAtGpsUpdate: FloatArray? = null //上記GPSデータで更新した直後のEKF位置
    private val GPS_CONSISTENCY_DISTANCE_THRESHOLD = 3.0f //m(EKFがこれ以上動いたのにGPSが動かない場合に補正) bf5
    private val GPS_STATIONARY_DISTANCE_RATIO = 0.75f      //GPS位置変化の許容範囲(accuracy * ratio)より小さければ静止とみなす

    //高速移動モード用に追加
    private var isHighSpeedMode = false // 高速移動モードフラグ
    private val HIGH_SPEED_THRESHOLD_KMH = 5.0f // km/h
    private val HIGH_SPEED_THRESHOLD_MPS = HIGH_SPEED_THRESHOLD_KMH / 3.6f // m/s

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // TextViewの紐付け
        textViewInfo = findViewById(R.id.textViewInfo)
        textViewX = findViewById(R.id.textViewX)
        textViewY = findViewById(R.id.textViewY)
        textViewZ = findViewById(R.id.textViewZ)
        textViewWarning = findViewById(R.id.textViewWarning)
        textViewVelocity = findViewById(R.id.textViewVelocity) // m/s 表示用
        textViewDistance = findViewById(R.id.textViewDistance)
        textViewWorldAcc = findViewById(R.id.textViewWorldAcc)
        textViewVelocityh = findViewById(R.id.textViewVelocityh) // km/h 表示用

        //EKFインスタンスの初期化
        ekf = ExtendedKalmanFilter()

        //GPSマネージャーの初期化とデータ監視設定
        gpsManager = GpsManager(this)
        lifecycleScope.launch {
            //Flowを収集し、新しいデータが来るたびに処理
            gpsManager.locationDataFlow.collect { locationData ->
                latestGpsLocationData = locationData //最新のGPSデータを常に保持
                if (locationData != null) {
                    Log.d("MainActivity", "Received GPS Data: Valid=${locationData.isLocalValid}, LocalX=${locationData.localX}, LocalY=${locationData.localY}, Acc=${locationData.accuracy}")
                    updateGpsUI(locationData)

                    //高速移動モードの更新
                    val currentGpsSpeed = locationData.speed
                    if (currentGpsSpeed != null && locationData.isLocalValid && (locationData.accuracy ?: Float.MAX_VALUE) < 30f ) { // 速度があり、有効で精度が良い場合
                        if (currentGpsSpeed > HIGH_SPEED_THRESHOLD_MPS) {
                            if (!isHighSpeedMode) {
                                Log.i("SpeedMode", "Entered High Speed Mode (GPS Speed: %.2f m/s)".format(currentGpsSpeed))
                                isHighSpeedMode = true
                            }
                        } else {
                            if (isHighSpeedMode) {
                                Log.i("SpeedMode", "Exited High Speed Mode (GPS Speed: %.2f m/s)".format(currentGpsSpeed))
                                isHighSpeedMode = false
                            }
                        }
                    } else {
                        //GPS速度が利用できない、または無効/精度が悪ければ高速モードを解除
                        if (isHighSpeedMode) {
                            Log.i("SpeedMode", "Exited High Speed Mode (Invalid GPS speed or accuracy)")
                            isHighSpeedMode = false
                        }
                    }

                    updateGpsUI(locationData) // UI更新
                } else {
                    // GPSデータがない場合は高速モード解除
                    if (isHighSpeedMode) {
                        Log.i("SpeedMode", "Exited High Speed Mode (GPS stopped)")
                        isHighSpeedMode = false
                    }

                }
            }
        }


        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager

        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        linearAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        if (rotationVectorSensor == null) {
            rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
        }

        //センサー情報の表示
        var sensorInfoText = ""
        sensorInfoText += if (gyroscope != null) "ジャイロ: OK\n" else "ジャイロ: NG\n"
        sensorInfoText += if (linearAccelerometer != null) "線形加速度: OK\n" else "線形加速度: NG\n"
        sensorInfoText += if (rotationVectorSensor != null) {
            if (rotationVectorSensor?.type == Sensor.TYPE_ROTATION_VECTOR) "回転ベクトル: OK\n"
            else "ゲーム回転ベクトル: OK\n"
        } else "回転センサー: NG\n"
        if (linearAccelerometer == null || rotationVectorSensor == null) {
            sensorInfoText += "速度/距離計算に必要なセンサーが不足しています。"
        }
        textViewInfo.text = sensorInfoText
        textViewWarning.text = "注意: EKFによる推定。静止状態検出で補正。" //メッセージ変更

        //初期化処理
        isGyroFirstTime = true
        isAccelFirstTime = true
        rotationMatrix[0] = 1f; rotationMatrix[4] = 1f; rotationMatrix[8] = 1f; //単位行列で初期化
    }

    override fun onResume() {
        super.onResume()
        //センサーリスナー登録
        gyroscope?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
            //isGyroFirstTime = true //resetMotionDataでリセットされるので不要かも
        }
        linearAccelerometer?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
            //isAccelFirstTime = true //resetMotionData でリセットされるので不要かも
        }
        rotationVectorSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
            //hasRotationVector = false //resetMotionData に含めるか検討
        }
        //位置情報取得開始
        gpsManager.startLocationUpdates()

        //再開時にリセット
        resetMotionData()
    }

    override fun onPause() {
        super.onPause()
        //センサーリスナー解除
        sensorManager.unregisterListener(this)
        //位置情報取得停止
        gpsManager.stopLocationUpdates()
    }

    //モーションデータ（EKF状態）リセット
    private fun resetMotionData() {
        if (::ekf.isInitialized) {
            ekf.reset()
        }
        worldAccel.fill(0f)
        isStationary = false
        angleX = 0f; angleY = 0f; angleZ = 0f
        gyroLastTimestamp = 0L
        accelLastTimestamp = 0L
        isGyroFirstTime = true //リセット時に true に戻す
        isAccelFirstTime = true //リセット時に true に戻す
        hasRotationVector = false //リセット時に false に戻すのが安全かも
        rotationMatrix[0] = 1f; rotationMatrix[4] = 1f; rotationMatrix[8] = 1f; //回転行列もリセット

        //GPS関連の履歴もリセット
        latestGpsLocationData = null
        lastValidGpsData = null
        lastEkfPositionAtGpsUpdate = null

        isHighSpeedMode = false // 高速モードフラグもリセット
        Log.d("MainActivity", "Motion data, GPS history, and speed mode reset.")

        Log.d("MainActivity", "Motion data and GPS history reset.")
    }


    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        //必要に応じて精度変更時の処理を記述
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return

        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> processGyroscope(event) //呼び出し
            Sensor.TYPE_LINEAR_ACCELERATION -> processLinearAcceleration(event)
            Sensor.TYPE_ROTATION_VECTOR, Sensor.TYPE_GAME_ROTATION_VECTOR -> processRotationVector(event) // ★★★ 呼び出し
        }
    }

    //ジャイロ処理(角度積分はそのまま、EKFとは独立して表示用に使う)
    private fun processGyroscope(event: SensorEvent) {
        currentGyroRate[0] = event.values[0]
        currentGyroRate[1] = event.values[1]
        currentGyroRate[2] = event.values[2]

        if (isGyroFirstTime) { //isGyroFirstTimeを参照
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

    //回転ベクトル処理(回転行列を更新)
    private fun processRotationVector(event: SensorEvent) {
        val rotationVector = event.values
        try {
            //rotationMatrixを参照・更新
            SensorManager.getRotationMatrixFromVector(rotationMatrix, rotationVector)
            hasRotationVector = true
        } catch (e: IllegalArgumentException) {
            Log.e("RotationVector", "IllegalArgumentException from getRotationMatrixFromVector: ${e.message}")
            hasRotationVector = false
        } catch (e: Exception) {
            Log.e("RotationVector", "Error processing Rotation Vector event: ${e.message}", e)
            hasRotationVector = false
        }
    }


    //線形加速度処理(EKFの予測と更新、補正を実行)
    private fun processLinearAcceleration(event: SensorEvent) {
        //isAccelFirstTimeとrotationMatrixを参照
        if (!hasRotationVector) return

        //dt計算
        if (isAccelFirstTime) { // ★★★ isAccelFirstTime を参照 ★★★
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
        //rotationMatrixを参照
        worldAccel[0] = rotationMatrix[0] * deviceAccelX + rotationMatrix[1] * deviceAccelY + rotationMatrix[2] * deviceAccelZ
        worldAccel[1] = rotationMatrix[3] * deviceAccelX + rotationMatrix[4] * deviceAccelY + rotationMatrix[5] * deviceAccelZ
        worldAccel[2] = rotationMatrix[6] * deviceAccelX + rotationMatrix[7] * deviceAccelY + rotationMatrix[8] * deviceAccelZ

        //EKF予測ステップ&静止更新
        val accelMagnitude = sqrt(worldAccel[0] * worldAccel[0] + worldAccel[1] * worldAccel[1] + worldAccel[2] * worldAccel[2])
        val gyroMagnitude = sqrt(currentGyroRate[0] * currentGyroRate[0] + currentGyroRate[1] * currentGyroRate[1] + currentGyroRate[2] * currentGyroRate[2])

        // GPS速度を取得（存在しない場合は十分に小さい値として扱う）
        val currentGpsSpeedMps = latestGpsLocationData?.speed ?: 0.0f
        // GPS速度が高速閾値未満かどうかを判定
        val isGpsSpeedBelowThreshold = currentGpsSpeedMps < HIGH_SPEED_THRESHOLD_MPS

        // 静止判定条件: IMUの動きが小さく、**かつ** GPS速度も高速閾値未満
        val isImuStationary = accelMagnitude < stationaryAccelThreshold && gyroMagnitude < stationaryGyroThreshold
        val isConsideredStationary = isImuStationary && isGpsSpeedBelowThreshold

        // 状態遷移ロジック
        val justBecameStationary = !isStationary && isConsideredStationary
        // 静止維持条件もGPS速度を考慮 (ヒステリシスはIMUのみにかける、または両方にかけるか要検討)
        // 例: IMUの閾値を少し緩め、GPS速度は閾値未満
        val isImuNearlyStationary = accelMagnitude < stationaryAccelThreshold * 1.5f && gyroMagnitude < stationaryGyroThreshold * 1.5f
        val remainsStationary = isStationary && isImuNearlyStationary && isGpsSpeedBelowThreshold

        if (justBecameStationary || remainsStationary) {
            if (!isStationary) {
                Log.i("Stationary", "Detected stationary state (IMU stationary AND GPS speed < %.1f km/h)".format(HIGH_SPEED_THRESHOLD_KMH))
                isStationary = true
            }
            // 静止状態の場合、EKFの更新ステップを実行
            ekf.updateStationary()
            // 静止時は加速度ゼロとして予測
            ekf.predict(dt, floatArrayOf(0f, 0f, 0f))
        } else {
            // isConsideredStationary が false の場合 (IMUが動いている or GPSが高速)
            if (isStationary) {
                Log.i("Stationary", "Exited stationary state (Reason: IMU moving OR GPS speed >= %.1f km/h)".format(HIGH_SPEED_THRESHOLD_KMH))
                isStationary = false
            }
            // 通常のEKF予測ステップを実行
            ekf.predict(dt, worldAccel)
        }

        //EKF更新前の位置を保持 (GPS矛盾チェック用)
        val ekfPositionBeforeGpsUpdate = ekf.getPosition().clone()

        //GPSデータによるEKF更新
        val gpsDataToUse = latestGpsLocationData
        var gpsUpdated = false
        if (gpsDataToUse != null && gpsDataToUse.isLocalValid) {
            if (lastValidGpsData == null || gpsDataToUse.timestamp > lastValidGpsData!!.timestamp) {
                Log.d("EKF_Update", "Attempting EKF update with GPS data (Timestamp: ${gpsDataToUse.timestamp})")
                Log.d("EKF_Update", "Attempting EKF update with GPS data (HighSpeed: $isHighSpeedMode)")

                ekf.updateGpsPosition(gpsDataToUse, isHighSpeedMode)

                gpsUpdated = true

                //IMU/GPS矛盾検出と補正
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

                    if (distEkf > GPS_CONSISTENCY_DISTANCE_THRESHOLD && distGps < gpsStationaryThreshold) {
                        Log.w("IMU_GPS_Correction", "Inconsistency! EKF moved %.2fm, GPS moved %.2fm (threshold %.2fm). Correcting EKF position.".format(distEkf, distGps, gpsStationaryThreshold))

                        val correctedEkfPosition = ekf.getPosition()
                        correctedEkfPosition[0] = lastEkfPos[0] + (correctedEkfPosition[0] - lastEkfPos[0]) * 0.5f
                        correctedEkfPosition[1] = lastEkfPos[1] + (correctedEkfPosition[1] - lastEkfPos[1]) * 0.5f
                        if (correctedEkfPosition.size > 2 && lastEkfPos.size > 2 && gpsDataToUse.localZ != null && lastGps.localZ != null) {
                            val dzEkfCorrected = correctedEkfPosition[2] - lastEkfPos[2]
                            correctedEkfPosition[2] = lastEkfPos[2] + dzEkfCorrected * 0.5f
                        }

                        ekf.x[0, 0] = correctedEkfPosition[0]
                        ekf.x[1, 0] = correctedEkfPosition[1]
                        if (correctedEkfPosition.size > 2) {
                            ekf.x[2, 0] = correctedEkfPosition[2]
                        }
                        Log.w("IMU_GPS_Correction", "Corrected EKF pos: X=%.2f, Y=%.2f, Z=%.2f".format(ekf.x[0, 0], ekf.x[1, 0], ekf.x[2, 0]))
                    }
                }

                //今回の更新に使ったGPSデータと、その直後のEKF位置を保存
                lastValidGpsData = gpsDataToUse
                lastEkfPositionAtGpsUpdate = ekf.getPosition().clone() // clone() で値をコピー

            }
        }

        //低速度域での速度クリッピング
        val currentVelocity = ekf.getVelocity()
        val currentSpeed = sqrt(currentVelocity[0] * currentVelocity[0] +
                currentVelocity[1] * currentVelocity[1] +
                currentVelocity[2] * currentVelocity[2])
        if (currentSpeed < MIN_VELOCITY_THRESHOLD) {
            if (ekf.x[3, 0] != 0f || ekf.x[4, 0] != 0f || ekf.x[5, 0] != 0f) {
                Log.d("SpeedClip", "Clipping speed below threshold (${MIN_VELOCITY_THRESHOLD} m/s). Speed: %.4f m/s".format(currentSpeed))
                ekf.x[3, 0] = 0f; ekf.x[4, 0] = 0f; ekf.x[5, 0] = 0f
            }
        }


        //UI更新
        updateUI()
    }


    private fun updateUI() {
        val ekfState = ekf.getState()
        val position = floatArrayOf(ekfState[0], ekfState[1], ekfState[2])
        val velocity = floatArrayOf(ekfState[3], ekfState[4], ekfState[5])

        runOnUiThread {
            //角度表示(ジャイロ積分)
            textViewX.text = "角度X: %.1f °".format(angleX)
            textViewY.text = "角度Y: %.1f °".format(angleY)
            textViewZ.text = "角度Z: %.1f °".format(angleZ)

            //ワールド座標系の加速度(EKFへの入力、デバッグ用)
            textViewWorldAcc.text = "加速(W): X:%.2f Y:%.2f Z:%.2f".format(worldAccel[0], worldAccel[1], worldAccel[2])

            //EKFによる速度(m/s)
            val worldSpeed = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2])
            textViewVelocity.text = "速度: %.2f m/s".format(worldSpeed)

            //時速(km/h)
            val worldSpeedKmh = worldSpeed * 3.6f
            textViewVelocityh.text = "時速: %.1f km/h".format(worldSpeedKmh)

            //EKFによる変位(m)
            val totalWorldDistance = sqrt(position[0] * position[0] + position[1] * position[1] + position[2] * position[2])
            textViewDistance.text = "変位(中): %.2f m (X:%.2f Y:%.2f Z:%.2f)".format(totalWorldDistance, position[0], position[1], position[2])

            //静止状態表示
            if (isStationary) {
                textViewWarning.text = "状態: 静止 (EKF更新中)"
                textViewWarning.setTextColor(getColor(android.R.color.holo_green_dark))
            } else {
                // 高速モードを優先表示
                if (isHighSpeedMode) {
                    textViewWarning.text = "状態: 高速移動 (GPS優先)"
                    textViewWarning.setTextColor(getColor(android.R.color.holo_red_light)) // 目立つ色に
                } else if (worldSpeed < MIN_VELOCITY_THRESHOLD) {
                    textViewWarning.text = "状態: 低速 (速度クリップ中)"
                    textViewWarning.setTextColor(getColor(android.R.color.holo_blue_dark))
                } else {
                    textViewWarning.text = "状態: 移動中 (EKF予測中)"
                    textViewWarning.setTextColor(getColor(android.R.color.holo_orange_dark))
                }
            }
        }
    }

    //(オプション)GPS情報を表示するUI更新メソッド
    private fun updateGpsUI(data: LocationData) {
        runOnUiThread {
            //例: TextView (gpsInfoTextView) をレイアウトに追加して表示
            //findViewById<TextView>(R.id.gpsInfoTextView)?.text = """
            //     GPS: ${if(data.isLocalValid) "有効" else "無効"} (Acc: ${data.accuracy?.format(1)}m)
            //     位置(W): X=${data.localX?.format(2)}, Y=${data.localY?.format(2)}, Z=${data.localZ?.format(2)}
            //     速度(W): ${data.speed?.format(2)} m/s
            //""".trimIndent()
        }
    }

    //補助関数(Float?のフォーマット用)
    fun Float?.format(digits: Int) = this?.let { "%.${digits}f".format(it) } ?: "N/A"


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