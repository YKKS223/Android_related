package com.example.test_gyro_1

import android.content.Context
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.TextView
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.example.test_gyro_1.gps.GpsManager
import com.example.test_gyro_1.processor.MotionProcessor
import com.example.test_gyro_1.processor.MotionState
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import com.example.test_gyro_1.view.PathView
import java.util.ArrayList

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var gyroscope: Sensor? = null
    private var linearAccelerometer: Sensor? = null
    private var rotationVectorSensor: Sensor? = null

    //UI Elements
    private lateinit var textViewInfo: TextView
    private lateinit var textViewPitch: TextView //X軸周り
    private lateinit var textViewRoll: TextView  //Y軸周り
    private lateinit var textViewYaw: TextView //Z軸周り
    private lateinit var textViewWarning: TextView
    private lateinit var textViewVelocity: TextView
    private lateinit var textViewDistance: TextView
    private lateinit var textViewWorldAcc: TextView
    private lateinit var textViewVelocityh: TextView
    private lateinit var textViewAccumulatedDistance: TextView
    private lateinit var buttonStopMeasurement: Button
    private lateinit var pathView: PathView

    //Managers and Processors
    private lateinit var gpsManager: GpsManager
    private lateinit var motionProcessor: MotionProcessor

    //状態管理フラグは不要

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        //TextViewとButtonの紐付け
        bindViews()

        //初期化
        motionProcessor = MotionProcessor() //プロセッサー初期化
        //reset() は onResume で行うか、前のActivityから来たことを示す Intent Extra で制御する方が良いかも
        //motionProcessor.reset() //onCreateでリセットするか検討
        gpsManager = GpsManager(this)
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        findSensors()
        displaySensorInfo()

        //ボタンリスナー設定
        buttonStopMeasurement.setOnClickListener { stopMeasurementAndShowResult() }

        //MotionProcessorの状態を監視してUI更新
        lifecycleScope.launch {
            motionProcessor.motionStateFlow.collectLatest { state ->
                updateUI(state) //常にUI更新
            }
        }

        // GPSデータ収集 MotionProcessorへ
        lifecycleScope.launch {
            gpsManager.locationDataFlow.collect { locationData ->
                motionProcessor.processGpsData(locationData) // 常に処理
            }
        }

        //初期UI状態設定
        //updateInitialUIState() //待機表示用のメソッドは不要に
        textViewWarning.text = "状態: 計測中" // 初期表示
        textViewWarning.setTextColor(ContextCompat.getColor(this, android.R.color.holo_orange_dark))
        //他のTextViewも初期値（0.0など）を表示しておく
        updateUI(MotionState.initial()) //MotionState の初期値でUIを初期化
    }

    override fun onResume() {
        super.onResume()
        //onResume でセンサーリスナー登録とGPS開始
        //前の画面から遷移してきた時や、バックグラウンドから復帰した時に計測を開始/再開
        Log.d("MainActivity", "onResume: Starting sensors and GPS")
        //motionProcessor をリセットするかどうか決定する
        //motionProcessor.reset() //必要ならここでリセット

        registerSensorListeners()
        gpsManager.startLocationUpdates()
    }

    override fun onPause() {
        super.onPause()
        Log.d("MainActivity", "onPause: Stopping sensors and GPS")
        unregisterSensorListeners()
        gpsManager.stopLocationUpdates()
    }

    private fun bindViews() {
        textViewInfo = findViewById(R.id.textViewInfo)
        textViewPitch = findViewById(R.id.textViewPitch) // レイアウトのIDも変更推奨
        textViewRoll = findViewById(R.id.textViewRoll)   // レイアウトのIDも変更推奨
        textViewYaw = findViewById(R.id.textViewYaw)
        textViewWarning = findViewById(R.id.textViewWarning)
        textViewVelocity = findViewById(R.id.textViewVelocity)
        textViewDistance = findViewById(R.id.textViewDistance)
        textViewWorldAcc = findViewById(R.id.textViewWorldAcc)
        textViewVelocityh = findViewById(R.id.textViewVelocityh)
        textViewAccumulatedDistance = findViewById(R.id.textViewAccumulatedDistance)
        buttonStopMeasurement = findViewById(R.id.buttonStopMeasurement)
        pathView = findViewById(R.id.pathView)
    }

    private fun findSensors() {
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        linearAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
            ?: sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
    }

    private fun displaySensorInfo() {
    }


    //計測終了処理 & 結果表示
    private fun stopMeasurementAndShowResult() {
        Log.i("MainActivity", "Measurement Stopped (Button Clicked)")

        //onPauseで停止しているはずだが、念のため再度停止
        unregisterSensorListeners()
        gpsManager.stopLocationUpdates()

        //最終結果を取得
        val finalState = motionProcessor.motionStateFlow.value

        //状態表示を更新
        textViewWarning.text = "状態: 計測終了"
        textViewWarning.setTextColor(ContextCompat.getColor(this, android.R.color.darker_gray))
        buttonStopMeasurement.isEnabled = false //ボタンを無効化

        val pathX = finalState.pathHistory.map { it.x }.toFloatArray()
        val pathY = finalState.pathHistory.map { it.y }.toFloatArray()


        //ResultActivityにデータを渡して起動
        val intent = Intent(this, ResultActivity::class.java).apply {
            putExtra(ResultActivity.EXTRA_ANGLE_X, finalState.anglePitch)
            putExtra(ResultActivity.EXTRA_ANGLE_Y, finalState.angleRoll)
            putExtra(ResultActivity.EXTRA_ANGLE_Z, finalState.angleYaw)
            putExtra(ResultActivity.EXTRA_DISPLACEMENT, finalState.totalDistance)
            putExtra(ResultActivity.EXTRA_TOTAL_DISTANCE, finalState.accumulatedDistance)
            putExtra(ResultActivity.EXTRA_PATH_X, pathX)
            putExtra(ResultActivity.EXTRA_PATH_Y, pathY)
        }

        finalState.currentPoint?.let { cp ->
            intent.putExtra(ResultActivity.EXTRA_CURRENT_POINT_X, cp.x)
            intent.putExtra(ResultActivity.EXTRA_CURRENT_POINT_Y, cp.y)
        }

        startActivity(intent)
    }

    private fun registerSensorListeners() {
        gyroscope?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        linearAccelerometer?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        rotationVectorSensor?.let { sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        Log.d("MainActivity", "Sensor listeners registered.")
    }

    private fun unregisterSensorListeners() {
        sensorManager.unregisterListener(this)
        Log.d("MainActivity", "Sensor listeners unregistered.")
    }


    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // 変更なし
        Log.d("MainActivity", "Sensor ${sensor?.name} accuracy changed to $accuracy")
    }

    override fun onSensorChanged(event: SensorEvent?) {
        //常に処理
        if (event == null) return

        //センサーイベントをMotionProcessorに渡す
        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> motionProcessor.processGyroscopeEvent(event)
            Sensor.TYPE_LINEAR_ACCELERATION -> motionProcessor.processLinearAccelerationEvent(event)
            Sensor.TYPE_ROTATION_VECTOR, Sensor.TYPE_GAME_ROTATION_VECTOR -> motionProcessor.processRotationVectorEvent(event)
        }
    }

    //UI更新メソッド
    private fun updateUI(state: MotionState) {
        runOnUiThread {
            //機種基準
            /*
            //Pitch (X軸周り): MotionStateのanglePitchを表示
            textViewPitch.text = "Pitch (X): %.1f °".format(state.anglePitch)

            //Roll (Y軸周り): MotionStateのangleRollを表示
            textViewRoll.text = "Roll (Y): %.1f °".format(state.angleRoll)

            //Yaw (Z軸周り): MotionStateのangleYawを表示
            textViewYaw.text = "Yaw (Z): %.1f °".format(state.angleYaw)

             */

            //画面基準
            textViewYaw.text = "X軸: %.1f °".format(state.angleYaw)
            textViewRoll.text = "Y軸: %.1f °".format(state.angleRoll)
            textViewPitch.text = "Z軸: %.1f °".format(state.anglePitch)

            //他のUI要素の更新
            textViewWorldAcc.text = "加速(W): X:%.2f Y:%.2f Z:%.2f".format(state.worldAccel[0], state.worldAccel[1], state.worldAccel[2])
            textViewVelocity.text = "速度: %.2f m/s".format(state.speedMps)
            textViewVelocityh.text = "時速: %.1f km/h".format(state.speedKmh)
            textViewDistance.text = "変位(直): %.2f m".format(state.totalDistance) //直線距離
            textViewAccumulatedDistance.text = "総移動距離: %.2f m".format(state.accumulatedDistance) //総合距離
            pathView.updatePath(state.pathHistory, state.currentPoint)

            //状態表示テキストと色
            textViewWarning.text = state.statusText
            //state.statusColorはandroid.R.colorのリソースIDなのでContextCompatを使う
            try {
                textViewWarning.setTextColor(ContextCompat.getColor(this, state.statusColor))
            } catch (e: Exception) {
                Log.e("MainActivity", "Failed to set text color for warning TextView. Color resource ID: ${state.statusColor}", e)
                //fallback color or default
                textViewWarning.setTextColor(ContextCompat.getColor(this, android.R.color.darker_gray))
            }
        }
    }
}