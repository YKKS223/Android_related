package com.example.test_gyro_1

import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.graphics.PointF
import android.widget.Button
import android.widget.TextView
import com.example.test_gyro_1.view.PathView
import android.util.Log
import java.util.ArrayList

class ResultActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_ANGLE_X = "com.example.test_gyro_1.EXTRA_ANGLE_X"
        const val EXTRA_ANGLE_Y = "com.example.test_gyro_1.EXTRA_ANGLE_Y"
        const val EXTRA_ANGLE_Z = "com.example.test_gyro_1.EXTRA_ANGLE_Z"
        const val EXTRA_DISPLACEMENT = "com.example.test_gyro_1.EXTRA_DISPLACEMENT"
        const val EXTRA_TOTAL_DISTANCE = "com.example.test_gyro_1.EXTRA_TOTAL_DISTANCE"
        const val EXTRA_PATH_X = "com.example.test_gyro_1.EXTRA_PATH_X"
        const val EXTRA_PATH_Y = "com.example.test_gyro_1.EXTRA_PATH_Y"
        const val EXTRA_CURRENT_POINT_X = "com.example.test_gyro_1.EXTRA_CURRENT_POINT_X"
        const val EXTRA_CURRENT_POINT_Y = "com.example.test_gyro_1.EXTRA_CURRENT_POINT_Y"
        const val DEFAULT_POINT_VALUE = Float.NEGATIVE_INFINITY
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_result)

        val textViewAngle: TextView = findViewById(R.id.textViewFinalAngleValue)
        val textViewDisplacement: TextView = findViewById(R.id.textViewDisplacementValue)
        val textViewTotalDistance: TextView = findViewById(R.id.textViewTotalDistanceValue)
        val resultPathView: PathView = findViewById(R.id.resultPathView)
        val buttonClose: Button = findViewById(R.id.buttonCloseResult)

        //Intentからデータを取得
        val angleX = intent.getFloatExtra(EXTRA_ANGLE_X, 0f)
        val angleY = intent.getFloatExtra(EXTRA_ANGLE_Y, 0f)
        val angleZ = intent.getFloatExtra(EXTRA_ANGLE_Z, 0f)
        val displacement = intent.getFloatExtra(EXTRA_DISPLACEMENT, 0f)
        val totalDistance = intent.getFloatExtra(EXTRA_TOTAL_DISTANCE, 0f)

        val pathXArray = intent.getFloatArrayExtra(EXTRA_PATH_X)
        val pathYArray = intent.getFloatArrayExtra(EXTRA_PATH_Y)

        val currentX = intent.getFloatExtra(EXTRA_CURRENT_POINT_X, DEFAULT_POINT_VALUE)
        val currentY = intent.getFloatExtra(EXTRA_CURRENT_POINT_Y, DEFAULT_POINT_VALUE)
        val currentPoint: PointF? = if (currentX != DEFAULT_POINT_VALUE && currentY != DEFAULT_POINT_VALUE) {
            PointF(currentX, currentY)
        } else {
            null // データが渡されなかった場合
        }


        //TextViewに表示
        textViewAngle.text = "X: %.1f°, Y: %.1f°, Z: %.1f°".format(angleX, angleY, angleZ)
        textViewDisplacement.text = "%.2f m".format(displacement)
        textViewTotalDistance.text = "%.2f m".format(totalDistance)

        val pathHistory = mutableListOf<PointF>()
        if (pathXArray != null && pathYArray != null && pathXArray.size == pathYArray.size) {
            for (i in pathXArray.indices) {
                pathHistory.add(PointF(pathXArray[i], pathYArray[i]))
            }
            Log.d("ResultActivity", "Path history received with ${pathHistory.size} points.")
            resultPathView.post {
                resultPathView.updatePath(pathHistory, currentPoint)
            }
        } else {
            Log.w("ResultActivity", "Path history data is missing or invalid.")
            if (currentPoint != null) {
                resultPathView.post {
                    resultPathView.updatePath(emptyList(), currentPoint)
                }
            }
        }

        //閉じるボタンの処理
        buttonClose.setOnClickListener {
            finish() //Activityを閉じる
        }
    }
}