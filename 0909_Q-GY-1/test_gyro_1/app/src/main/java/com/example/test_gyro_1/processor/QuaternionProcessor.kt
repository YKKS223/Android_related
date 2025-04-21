package com.example.test_gyro_1.processor

import android.hardware.SensorEvent
import android.hardware.SensorManager
import android.util.Log
import kotlin.math.*

class QuaternionProcessor {

    private var currentRotationVector: FloatArray? = null
    private val quaternion = FloatArray(4) // [x, y, z, w]
    private val rotationMatrix = FloatArray(9)
    private val orientationAnglesRad = FloatArray(3)
    private val orientationAnglesDeg = FloatArray(3)

    fun getRotationMatrix(): FloatArray? {
        // currentRotationVector が null でないことを確認
        // (processRotationVectorEvent が少なくとも1回呼ばれ、有効なデータを処理したか)
        return if (currentRotationVector != null) {
            // 内部の行列が変更されないようにコピーを返す
            rotationMatrix.clone()
        } else {
            // まだ有効な回転行列が計算されていない
            null
        }
    }

    fun processRotationVectorEvent(event: SensorEvent) {
        if (event.values.size >= 4) {
            try {
                //Android の Quaternion: [x, y, z, w]
                SensorManager.getQuaternionFromVector(quaternion, event.values)
                currentRotationVector = event.values.clone()

                //独自実装による回転行列計算
                getRotationMatrixFromQuaternion(quaternion, rotationMatrix)

                SensorManager.getOrientation(rotationMatrix, orientationAnglesRad)

                orientationAnglesDeg[0] = Math.toDegrees(orientationAnglesRad[0].toDouble()).toFloat() //Yaw
                orientationAnglesDeg[1] = Math.toDegrees(orientationAnglesRad[1].toDouble()).toFloat() //Pitch
                orientationAnglesDeg[2] = Math.toDegrees(orientationAnglesRad[2].toDouble()).toFloat() //Roll

                orientationAnglesDeg[0] = normalizeAngle(orientationAnglesDeg[0])

            } catch (e: IllegalArgumentException) {
                Log.e("QuaternionProcessor", "Error converting rotation vector: ${e.message}")
                resetAngles()
            } catch (e: Exception) {
                Log.e("QuaternionProcessor", "Unexpected error processing rotation vector", e)
                resetAngles()
            }
        } else {
            Log.w("QuaternionProcessor", "Received rotation vector event with unexpected size: ${event.values.size}")
            resetAngles()
        }
    }

    fun getEulerAnglesDegrees(): FloatArray {
        return orientationAnglesDeg.clone()
    }

    fun reset() {
        currentRotationVector = null
        quaternion.fill(0f)
        rotationMatrix.fill(0f)
        orientationAnglesRad.fill(0f)
        orientationAnglesDeg.fill(0f)
        Log.d("QuaternionProcessor", "Quaternion state reset.")
    }

    private fun resetAngles() {
        orientationAnglesRad.fill(0f)
        orientationAnglesDeg.fill(0f)
    }

    private fun normalizeAngle(angle: Float): Float {
        var normalized = angle % 360
        if (normalized > 180) {
            normalized -= 360
        } else if (normalized <= -180) {
            normalized += 360
        }
        return normalized
    }


    private fun getRotationMatrixFromQuaternion(q: FloatArray, matrix: FloatArray) {
        val x = q[0]
        val y = q[1]
        val z = q[2]
        val w = q[3]

        val xx = x * x
        val yy = y * y
        val zz = z * z
        val xy = x * y
        val xz = x * z
        val yz = y * z
        val wx = w * x
        val wy = w * y
        val wz = w * z

        matrix[0] = 1f - 2f * (yy + zz)
        matrix[1] = 2f * (xy - wz)
        matrix[2] = 2f * (xz + wy)

        matrix[3] = 2f * (xy + wz)
        matrix[4] = 1f - 2f * (xx + zz)
        matrix[5] = 2f * (yz - wx)

        matrix[6] = 2f * (xz - wy)
        matrix[7] = 2f * (yz + wx)
        matrix[8] = 1f - 2f * (xx + yy)
    }
}
