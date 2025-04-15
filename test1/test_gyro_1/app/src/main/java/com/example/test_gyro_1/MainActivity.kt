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
import com.example.test_gyro_1.filter.ExtendedKalmanFilter
import com.example.test_gyro_1.filter.SimpleMatrix

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var gyroscope: Sensor? = null
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
    private val driftThreshold = 0.1f //bf0.01
    private var isGyroFirstTime = true
    private var currentGyroRate = FloatArray(3) //現在の角速度(rad/s)を保持

    // --- EKF関連 ---
    private lateinit var ekf: ExtendedKalmanFilter //EKFインスタンス
    private val worldAccel = FloatArray(3) //ワールド座標系の加速度(EKF入力用)

    //加速度センサーとセンサーフュージョン関連
    private var accelLastTimestamp: Long = 0
    private var isAccelFirstTime = true
    //静止状態検出のための閾値(調整が必要)
    private val stationaryAccelThreshold = 0.01f //m/s^2(この値以下の加速度の大きさは静止とみなす) bf0.1
    private val stationaryGyroThreshold = 0.01f //rad/s(この値以下の角速度の大きさは静止とみなす) bf0.02
    private var isStationary = false //静止状態フラグ

    //回転ベクトルセンサー関連
    private val rotationMatrix = FloatArray(9) //回転行列
    private val orientationAngles = FloatArray(3) //回転角度 (デバッグ用)
    // private var currentRotationVector = FloatArray(4) //不要になった
    private var hasRotationVector = false

    //EMAフィルター関連(不要に)
    //private val alpha = 0.1f
    //private var filteredWorldAccelX = 0f ... などは削除しました

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        //TextViewの紐付け
        textViewInfo = findViewById(R.id.textViewInfo)
        textViewX = findViewById(R.id.textViewX)
        textViewY = findViewById(R.id.textViewY)
        textViewZ = findViewById(R.id.textViewZ)
        textViewWarning = findViewById(R.id.textViewWarning)
        textViewVelocity = findViewById(R.id.textViewVelocity) //m/s 表示用
        textViewDistance = findViewById(R.id.textViewDistance)
        textViewWorldAcc = findViewById(R.id.textViewWorldAcc)
        textViewVelocityh = findViewById(R.id.textViewVelocityh) //km/h 表示用

        //EKFインスタンスの初期化
        ekf = ExtendedKalmanFilter()

        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager

        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        linearAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        if (rotationVectorSensor == null) {
            rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
        }

        //センサー情報の表示 (変更なし)
        var sensorInfoText = ""
        sensorInfoText += if (gyroscope != null) "ジャイロ: OK\n" else "ジャイロ: NG\n"
        sensorInfoText += if (linearAccelerometer != null) "線形加速度: OK\n" else "線形加速度: NG\n"
        sensorInfoText += if (rotationVectorSensor != null) {
            if (rotationVectorSensor?.type == Sensor.TYPE_ROTATION_VECTOR) "回転ベクトル: OK\n"
            else "ゲーム回転ベクトル: OK\n"
        } else "回転センサー: NG\n"
        if (linearAccelerometer == null || rotationVectorSensor == null) {
            sensorInfoText += "速度/距離計算に必要なセンサーが不足しています。"
            //...(表示クリア処理はそのまま)
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
        gyroscope?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
            gyroLastTimestamp = 0L
            isGyroFirstTime = true
            currentGyroRate.fill(0f)
        }
        linearAccelerometer?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
            accelLastTimestamp = 0L
            isAccelFirstTime = true
            resetMotionData() //EKFリセット含む
        }
        rotationVectorSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
            hasRotationVector = false
        }
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
    }

    // モーションデータ（EKF状態）リセット
    private fun resetMotionData() {
        if (::ekf.isInitialized) { //ekfが初期化済みか確認
            ekf.reset()
        }
        worldAccel.fill(0f)
        isStationary = false
        //ジャイロ積分の角度もリセットするかどうかは仕様による
        angleX = 0f; angleY = 0f; angleZ = 0f
    }


    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        //必要に応じて精度変更時の処理を記述
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return

        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> processGyroscope(event)
            Sensor.TYPE_LINEAR_ACCELERATION -> processLinearAcceleration(event)
            Sensor.TYPE_ROTATION_VECTOR, Sensor.TYPE_GAME_ROTATION_VECTOR -> processRotationVector(event)
        }
    }

    //ジャイロ処理(角度積分はそのまま、EKFとは独立して表示用に使う)
    private fun processGyroscope(event: SensorEvent) {
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
        if (dt <= 0) return
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

        //ここでのUI更新はしない(加速度イベントでまとめて行う)
    }

    //回転ベクトル処理(回転行列を更新)
    private fun processRotationVector(event: SensorEvent) {
        val rotationVector = if (event.values.size >= 4) {
            event.values //.copyOf(4) は不要な場合がある
        } else {
            Log.w("RotationVector", "Unexpected rotation vector size: ${event.values.size}")
            //エラーの場合、前回値を使うなどの処理が必要かもしれないが、一旦そのまま
            event.values
        }
        //getRotationMatrixFromVector は5要素以上必要な場合があるので注意
        if (rotationVector.size >= 4) {
            SensorManager.getRotationMatrixFromVector(rotationMatrix, rotationVector)
            //必要なら getOrientation も呼ぶ(デバッグ用)
            //SensorManager.getOrientation(rotationMatrix, orientationAngles)
            hasRotationVector = true
        } else if (rotationVector.size == 3 && event.sensor.type == Sensor.TYPE_GAME_ROTATION_VECTOR) {
            //Game Rotation Vectorが3要素の場合の処理(APIレベルによるかもしれない)
            // ドキュメント上は常にクォータニオンを返すはずだが、念のため
            Log.w("RotationVector", "Game Rotation Vector has only 3 values. Assuming quaternion without w.")
            //必要であれば、q.w = sqrt(1 - q.x^2 - q.y^2 - q.z^2)を計算して4要素にする
            //この例では単純化のため、取得できた値で回転行列を計算
            SensorManager.getRotationMatrixFromVector(rotationMatrix, rotationVector)
            hasRotationVector = true
        } else {
            Log.e("RotationVector", "Cannot get Rotation Matrix from vector size ${rotationVector.size}")
            hasRotationVector = false //回転行列が取得できなければフラグをfalseに
        }
    }

    //線形加速度処理 (EKFの予測と更新を実行)
    private fun processLinearAcceleration(event: SensorEvent) {
        if (!hasRotationVector) return //回転情報がないとワールド座標に変換できない

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
        if (dt <= 0) return
        accelLastTimestamp = event.timestamp

        //1.デバイス座標系の線形加速度を取得
        val deviceAccelX = event.values[0]
        val deviceAccelY = event.values[1]
        val deviceAccelZ = event.values[2]

        //2.ワールド座標系の加速度に変換
        //worldAccel = rotationMatrix * deviceAccel
        worldAccel[0] = rotationMatrix[0] * deviceAccelX + rotationMatrix[1] * deviceAccelY + rotationMatrix[2] * deviceAccelZ
        worldAccel[1] = rotationMatrix[3] * deviceAccelX + rotationMatrix[4] * deviceAccelY + rotationMatrix[5] * deviceAccelZ
        worldAccel[2] = rotationMatrix[6] * deviceAccelX + rotationMatrix[7] * deviceAccelY + rotationMatrix[8] * deviceAccelZ
        //※注意: 上記の計算は回転行列が列優先(OpenGLなど)の場合。AndroidのSensorManager.getRotationMatrixFromVectorは
        //ドキュメント上は明確でないが、通常は列優先で返すとされる。もし結果がおかしい場合は転置が必要かもしれない。
        //worldAccel[0] = rotationMatrix[0] * deviceAccelX + rotationMatrix[3] * deviceAccelY + rotationMatrix[6] * deviceAccelZ の形式が正しい場合もある。
        //(現在のコードでは後者の形式を使っていたので、そちらに合わせる)
        worldAccel[0] = rotationMatrix[0] * deviceAccelX + rotationMatrix[3] * deviceAccelY + rotationMatrix[6] * deviceAccelZ
        worldAccel[1] = rotationMatrix[1] * deviceAccelX + rotationMatrix[4] * deviceAccelY + rotationMatrix[7] * deviceAccelZ
        worldAccel[2] = rotationMatrix[2] * deviceAccelX + rotationMatrix[5] * deviceAccelY + rotationMatrix[8] * deviceAccelZ


        // --- 静止状態の検出 ---
        val accelMagnitude = sqrt(worldAccel[0] * worldAccel[0] + worldAccel[1] * worldAccel[1] + worldAccel[2] * worldAccel[2])
        val gyroMagnitude = sqrt(currentGyroRate[0] * currentGyroRate[0] + currentGyroRate[1] * currentGyroRate[1] + currentGyroRate[2] * currentGyroRate[2])

        val justBecameStationary = !isStationary && (accelMagnitude < stationaryAccelThreshold && gyroMagnitude < stationaryGyroThreshold)
        val remainsStationary = isStationary && (accelMagnitude < stationaryAccelThreshold * 1.5f && gyroMagnitude < stationaryGyroThreshold * 1.5f) // ヒステリシスを持たせる

        if (justBecameStationary || remainsStationary) {
            if (!isStationary) {
                Log.d("Stationary", "Detected stationary state.")
                isStationary = true
            }
            //静止状態の場合、EKFの更新ステップを実行
            ekf.updateStationary()
            //静止時は加速度をゼロとして予測するか、小さい値として予測するかは設計による
            //ここでは観測更新に任せ、予測はそのまま計算された加速度で行う
            ekf.predict(dt, floatArrayOf(0f, 0f, 0f)) //静止時は加速度ゼロとして予測
            //ekf.predict(dt, worldAccel) //計算された加速度で予測し、更新ステップで補正
        } else {
            if (isStationary) {
                Log.d("Stationary", "Exited stationary state.")
                isStationary = false
                //静止状態から抜けた直後は、EKFの共分散を少し大きくして不確かさを増やすことも考えられる
            }
            //通常EKF予測
            ekf.predict(dt, worldAccel)
        }

        updateUI()
    }


    private fun updateUI() {
        val ekfState = ekf.getState()
        val position = floatArrayOf(ekfState[0], ekfState[1], ekfState[2])
        val velocity = floatArrayOf(ekfState[3], ekfState[4], ekfState[5])

        runOnUiThread {
            //角度表示 (ジャイロ積分)
            textViewX.text = "角度X: %.1f °".format(angleX)
            textViewY.text = "角度Y: %.1f °".format(angleY)
            textViewZ.text = "角度Z: %.1f °".format(angleZ)

            //ワールド座標系の加速度(EKFへの入力、デバッグ用)
            textViewWorldAcc.text = "加速: X:%.2f Y:%.2f Z:%.2f".format(worldAccel[0], worldAccel[1], worldAccel[2])

            //EKFによる速度 (m/s)
            val worldSpeed = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2])
            textViewVelocity.text = "速度: %.2f m/s".format(worldSpeed)

            //時速(km/h)
            val worldSpeedKmh = worldSpeed * 3.6f
            textViewVelocityh.text = "時速: %.1f km/h".format(worldSpeedKmh)

            //EKFによる変位(m)
            val totalWorldDistance = sqrt(position[0] * position[0] + position[1] * position[1] + position[2] * position[2])
            textViewDistance.text = "変位(低): %.2f m (X:%.2f Y:%.2f Z:%.2f)".format(totalWorldDistance, position[0], position[1], position[2])

            //静止状態表示
            if (isStationary) {
                textViewWarning.text = "状態: 静止 (EKF更新中)"
                textViewWarning.setTextColor(getColor(android.R.color.holo_green_dark))
            } else {
                textViewWarning.text = "状態: 移動中 (EKF予測中)"
                textViewWarning.setTextColor(getColor(android.R.color.holo_orange_dark))
            }
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