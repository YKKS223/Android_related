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

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var gyroscope: Sensor? = null
    private var linearAccelerometer: Sensor? = null
    private var rotationVectorSensor: Sensor? = null

    //UI Elements
    private lateinit var textViewInfo: TextView
    private lateinit var textViewX: TextView
    private lateinit var textViewY: TextView
    private lateinit var textViewZ: TextView
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
        textViewX = findViewById(R.id.textViewX)
        textViewY = findViewById(R.id.textViewY)
        textViewZ = findViewById(R.id.textViewZ)
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

        //ResultActivityにデータを渡して起動
        val intent = Intent(this, ResultActivity::class.java).apply {
            putExtra(ResultActivity.EXTRA_ANGLE_X, finalState.angleX)
            putExtra(ResultActivity.EXTRA_ANGLE_Y, finalState.angleY)
            putExtra(ResultActivity.EXTRA_ANGLE_Z, finalState.angleZ)
            putExtra(ResultActivity.EXTRA_DISPLACEMENT, finalState.totalDistance)
            putExtra(ResultActivity.EXTRA_TOTAL_DISTANCE, finalState.accumulatedDistance)
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
            textViewX.text = "角度X: %.1f °".format(state.angleX)
            textViewY.text = "角度Y: %.1f °".format(state.angleY)
            textViewZ.text = "角度Z: %.1f °".format(state.angleZ)
            textViewWorldAcc.text = "加速(W): X:%.2f Y:%.2f Z:%.2f".format(state.worldAccel[0], state.worldAccel[1], state.worldAccel[2])
            textViewVelocity.text = "速度: %.2f m/s".format(state.speedMps)
            textViewVelocityh.text = "時速: %.1f km/h".format(state.speedKmh)
            textViewDistance.text = "変位(直): %.2f m".format(state.totalDistance) //直線距離
            textViewAccumulatedDistance.text = "総移動距離: %.2f m".format(state.accumulatedDistance) //総合距離
            pathView.updatePath(state.pathHistory, state.currentPoint)
            //textViewWarning.text = state.statusText
            //textViewWarning.setTextColor(ContextCompat.getColor(this, state.statusColor))
        }
    }
}