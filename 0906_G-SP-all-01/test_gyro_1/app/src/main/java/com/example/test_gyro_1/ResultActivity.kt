package com.example.test_gyro_1

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.Button
import android.widget.TextView

class ResultActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_ANGLE_X = "com.example.test_gyro_1.EXTRA_ANGLE_X"
        const val EXTRA_ANGLE_Y = "com.example.test_gyro_1.EXTRA_ANGLE_Y"
        const val EXTRA_ANGLE_Z = "com.example.test_gyro_1.EXTRA_ANGLE_Z"
        const val EXTRA_DISPLACEMENT = "com.example.test_gyro_1.EXTRA_DISPLACEMENT"
        const val EXTRA_TOTAL_DISTANCE = "com.example.test_gyro_1.EXTRA_TOTAL_DISTANCE"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_result)

        val textViewAngle: TextView = findViewById(R.id.textViewFinalAngleValue)
        val textViewDisplacement: TextView = findViewById(R.id.textViewDisplacementValue)
        val textViewTotalDistance: TextView = findViewById(R.id.textViewTotalDistanceValue)
        val buttonClose: Button = findViewById(R.id.buttonCloseResult)

        //Intentからデータを取得
        val angleX = intent.getFloatExtra(EXTRA_ANGLE_X, 0f)
        val angleY = intent.getFloatExtra(EXTRA_ANGLE_Y, 0f)
        val angleZ = intent.getFloatExtra(EXTRA_ANGLE_Z, 0f)
        val displacement = intent.getFloatExtra(EXTRA_DISPLACEMENT, 0f)
        val totalDistance = intent.getFloatExtra(EXTRA_TOTAL_DISTANCE, 0f)

        //TextViewに表示
        textViewAngle.text = "X: %.1f°, Y: %.1f°, Z: %.1f°".format(angleX, angleY, angleZ)
        textViewDisplacement.text = "%.2f m".format(displacement)
        textViewTotalDistance.text = "%.2f m".format(totalDistance)

        //閉じるボタンの処理
        buttonClose.setOnClickListener {
            finish() //Activityを閉じる
        }
    }
}