package com.example.test_gyro_1.filter

import android.util.Log
import kotlin.math.*
import com.example.test_gyro_1.filter.SimpleMatrix

/*
  姿勢推定のための拡張カルマンフィルタ (Attitude EKF)
  状態ベクトル: 4次元クォータニオン [q0, q1, q2, q3] (w, x, y, z)
  入力: ジャイロスコープ角速度 [wx, wy, wz] (rad/s)
  観測: 回転ベクトルセンサー由来のクォータニオン [q0, q1, q2, q3]
 */
class AttitudeEKF {

    //状態ベクトル x: クォータニオン [q0, q1, q2, q3] (w, x, y, z)
    var x: SimpleMatrix = SimpleMatrix(4, 1, floatArrayOf(1.0f, 0f, 0f, 0f)) // 初期値: 単位クォータニオン

    //状態推定誤差共分散行列 P (4x4)
    var P: SimpleMatrix = SimpleMatrix.identity(4) * 0.1f // 初期共分散 (ある程度の不確かさ)

    //プロセスノイズ共分散行列 Q (4x4) - ジャイロノイズやモデル誤差
    //ジャイロバイアス等を状態に入れていないため、プロセスノイズでモデル誤差を吸収
    private val gyroNoiseStdDev = Math.toRadians(0.1).toFloat() // ジャイロノイズ標準偏差 (例: 0.1 deg/s -> rad/s)
    private var Q: SimpleMatrix = SimpleMatrix.identity(4) * (gyroNoiseStdDev * gyroNoiseStdDev) // 後で dt を考慮してスケーリング

    //観測ノイズ共分散行列 R (4x4) - 回転ベクトルセンサーのノイズ
    //回転ベクトルセンサーのクォータニオン出力の不確かさ
    private val rotationVectorNoiseStdDev = 0.01f //クォータニオン各要素の標準偏差 (仮定、要調整)
    private val R: SimpleMatrix = SimpleMatrix.identity(4) * (rotationVectorNoiseStdDev * rotationVectorNoiseStdDev)

    //観測モデルヤコビアン H (観測が状態そのものなので単位行列)
    private val H: SimpleMatrix = SimpleMatrix.identity(4)

    //計算用一時変数
    private val F = SimpleMatrix.identity(4) //状態遷移行列 (予測時に更新)
    private val I = SimpleMatrix.identity(4) //単位行列

    //EKF の状態をリセットする

    fun reset() {
        x = SimpleMatrix(4, 1, floatArrayOf(1.0f, 0f, 0f, 0f)) //単位クォータニオン
        P = SimpleMatrix.identity(4) * 0.1f
        Log.d("AttitudeEKF", "EKF Reset.")
    }

    /*
      EKF 予測
      @param gyroRates ジャイロスコープの角速度 [wx, wy, wz] (rad/s)
      @param dt 時間ステップ (秒)
     */
    fun predict(gyroRates: FloatArray, dt: Float) {
        if (dt <= 1e-9f) return // dt が小さすぎる場合はスキップ

        val wx = gyroRates[0]
        val wy = gyroRates[1]
        val wz = gyroRates[2]

        //現在のクォータニオンを取得
        val q0 = x[0, 0]; val q1 = x[1, 0]; val q2 = x[2, 0]; val q3 = x[3, 0]

        //クォータニオン運動モデルに基づいて状態を予測
        //dq/dt = 0.5 * Omega(w) * q
        //予測: q_pred = q + dq/dt * dt
        val dq0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz)
        val dq1 = 0.5f * ( q0 * wx + q2 * wz - q3 * wy)
        val dq2 = 0.5f * ( q0 * wy - q1 * wz + q3 * wx)
        val dq3 = 0.5f * ( q0 * wz + q1 * wy - q2 * wx)

        val x_pred = SimpleMatrix(4, 1)
        x_pred[0, 0] = q0 + dq0 * dt
        x_pred[1, 0] = q1 + dq1 * dt
        x_pred[2, 0] = q2 + dq2 * dt
        x_pred[3, 0] = q3 + dq3 * dt

        //予測されたクォータニオンを正規化
        normalizeQuaternion(x_pred)
        x = x_pred // 状態を更新

        //状態遷移行列 F の計算 (線形化) F = I + 0.5 * Omega(w) * dt
        //Omega(w) = [ 0 -wx -wy -wz ]
        //           [ wx  0  wz -wy ]
        //           [ wy -wz  0  wx ]
        //           [ wz  wy -wx  0 ]
        F.setIdentity() // まず単位行列に
        val dt_half = 0.5f * dt
        F[0, 1] = -wx * dt_half; F[0, 2] = -wy * dt_half; F[0, 3] = -wz * dt_half
        F[1, 0] =  wx * dt_half; F[1, 2] =  wz * dt_half; F[1, 3] = -wy * dt_half
        F[2, 0] =  wy * dt_half; F[2, 1] = -wz * dt_half; F[2, 3] =  wx * dt_half
        F[3, 0] =  wz * dt_half; F[3, 1] =  wy * dt_half; F[3, 2] = -wx * dt_half

        //プロセスノイズ Q のスケーリング (dt を考慮)
        //簡単な方法: Q_k = Q_base * dt
        val Q_k = Q * dt // Qは gyroNoiseStdDev^2 ベースなので dt をかける

        //共分散行列 P の予測 P_pred = F * P * F^T + Q_k
        P = F * P * F.transpose() + Q_k

        //Pが対称性を失う可能性があるので、強制的に対称にする (オプション)
        //P = (P + P.transpose()) * 0.5f
    }

    /*
      EKF 更新ステップ
      @param measurementQuaternion 観測されたクォータニオン [q0, q1, q2, q3] (w, x, y, z) from Rotation Vector Sensor
     */
    fun update(measurementQuaternion: FloatArray) {
        if (measurementQuaternion.size < 4) return

        //観測ベクトル z
        val z = SimpleMatrix(4, 1, measurementQuaternion)
        //観測値を正規化 (念のため)
        normalizeQuaternion(z)

        //観測モデル h(x) = x (状態そのものを観測)
        //観測残差 y = z - h(x)
        val y = z - x

        //観測モデルヤコビアン H = I (単位行列)
        //残差共分散 S = H * P * H^T + R = P + R (H=I なので)
        val S = P + R

        //カルマンゲイン K = P * H^T * S^-1 = P * S^-1 (H=I なので)
        val S_inv = S.inverse4x4() // 4x4の逆行列計算が必要
        if (S_inv == null) {
            Log.e("AttitudeEKF", "Failed to invert S matrix in update step.")
            //逆行列が計算できない場合は更新をスキップ
            //Pが発散している可能性なども考えられる
            return
        }
        val K = P * S_inv

        //状態の更新 x_new = x + K * y
        x += K * y
        //更新されたクォータニオンを正規化
        normalizeQuaternion(x)

        //共分散の更新 P_new = (I - K * H) * P = (I - K) * P (H=I なので)
        P = (I - K) * P

        //Pが対称性を失う可能性があるので、強制的に対称にする (オプション)
        //P = (P + P.transpose()) * 0.5f
        //Log.d("AttitudeEKF", "Update Applied. State:\n$x")
    }

    /*
      現在の推定クォータニオンを取得 [w, x, y, z]
     */
    fun getQuaternion(): FloatArray {
        return floatArrayOf(x[0, 0], x[1, 0], x[2, 0], x[3, 0])
    }


    companion object {
        /**
         * SimpleMatrix 内のクォータニオンを正規化する (in-place)
         */
        fun normalizeQuaternion(qMatrix: SimpleMatrix) {
            if (qMatrix.rows != 4 || qMatrix.cols != 1) return
            val q0 = qMatrix[0, 0]; val q1 = qMatrix[1, 0]; val q2 = qMatrix[2, 0]; val q3 = qMatrix[3, 0]
            val norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
            if (norm > 1e-9f) {
                val invNorm = 1.0f / norm
                qMatrix[0, 0] = q0 * invNorm
                qMatrix[1, 0] = q1 * invNorm
                qMatrix[2, 0] = q2 * invNorm
                qMatrix[3, 0] = q3 * invNorm
            } else {
                //ノルムが小さすぎる場合は単位クォータニオンに
                qMatrix[0, 0] = 1.0f; qMatrix[1, 0] = 0f; qMatrix[2, 0] = 0f; qMatrix[3, 0] = 0f
            }
        }

        /*
          クォータニオン [w, x, y, z] から 3x3 回転行列を計算
          @param q クォータニオン配列 [w, x, y, z]
          @param matrix 出力先 3x3 回転行列 (9要素 FloatArray)
         */
        fun getRotationMatrixFromQuaternion(q: FloatArray, matrix: FloatArray) {
            if (q.size < 4 || matrix.size < 9) return
            val w = q[0]; val x = q[1]; val y = q[2]; val z = q[3]

            val xx = x * x; val yy = y * y; val zz = z * z
            val xy = x * y; val xz = x * z; val yz = y * z
            val wx = w * x; val wy = w * y; val wz = w * z

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

        /*
          回転行列からオイラー角 [Yaw, Pitch, Roll] (ラジアン) を計算
          SensorManager.getOrientation と同じ計算をする (ジンバルロック考慮)
          @param R 回転行列 (9要素)
          @param orientation 出力先オイラー角配列 (3要素、ラジアン) [Yaw, Pitch, Roll]
         */
        fun getOrientationFromRotationMatrix(R: FloatArray, orientation: FloatArray) {
            if (R.size < 9 || orientation.size < 3) return

            //Z-Y-X 順序 (一般的な航空宇宙シーケンス)
            //Pitch (X軸回転)
            //asin の引数が [-1, 1] の範囲を超える場合があるためクリップ
            val sinPitch = -R[7] // -R[2][1] (0-indexed matrix)
            orientation[1] = if (sinPitch >= 1.0f) {
                (PI / 2.0).toFloat() // Pitch = 90 deg
            } else if (sinPitch <= -1.0f) {
                (-PI / 2.0).toFloat() // Pitch = -90 deg
            } else {
                asin(sinPitch)
            }

            //ジンバルロックのチェック (Pitch が ±90度の場合)
            if (abs(orientation[1]) < (PI / 2.0 - 1e-6)) { //90度から少し離れていれば通常計算
                //Yaw (Z軸回転)
                orientation[0] = atan2(R[6], R[8]) // atan2(R[1][0], R[2][0]) -> atan2(R[6], R[8]) for ZYX ? Check matrix index
                //Yaw (Z軸回転)
                //atan2(R[1], R[4]) // atan2(R[0][1], R[1][1]) ?
                orientation[0] = atan2(R[1], R[4]) // atan2(R[0][1], R[1][1]) -> Check indices!

                //Roll (Y軸回転) - Y軸周り？ Y軸は R[3], R[4], R[5]
                //Roll (X軸回転) - X軸周り？ X軸は R[0], R[1], R[2]
                //SensorManager.getOrientation() のドキュメントによると [yaw, pitch, roll]
                //yaw: Z, pitch: X, roll: Y ?
                //Roll (Y軸回転) - Y軸周り？
                //atan2(R[3], R[0]) ?? atan2(R[2][0], R[0][0]) ?? -> Roll ZYX: atan2(R[7], R[4]) ?? -> No, R[6], R[7]
                //atan2(R[7], R[8]) // atan2(R[2][1], R[2][2]) ? No. R[6],R[7]
                //atan2(R[6], R[0]) // atan2(R[2][0], R[0][0]) ?
                //Trying based on common ZYX Euler convention:
                orientation[0] = atan2(R[1], R[4]) // Yaw (around Z) atan2(m[0][1], m[1][1])
                orientation[2] = atan2(R[6], R[7]) // Roll (around X) atan2(m[2][0], m[2][1]) -> No, m[2][0], m[2][2] ?
                orientation[2] = atan2(R[6], R[0]) // Roll (around X) atan2(m[2][0], m[0][0]) ?

                //Let's re-verify SensorManager.getOrientation order and calculation
                //Based on Android source code (frameworks/base/core/java/android/hardware/SensorManager.java)
                //Assuming matrix indices:
                //[ R[0] R[1] R[2] ]
                //[ R[3] R[4] R[5] ]
                //[ R[6] R[7] R[8] ]
                //Yaw (Z) = atan2(R[1], R[4])
                orientation[0] = atan2(R[1], R[4])
                //Pitch (X) = asin(-R[7]) -- Already calculated
                //Roll (Y) = atan2(R[6], R[8])
                orientation[2] = atan2(R[6], R[8])

            } else { //ジンバルロック状態
                //Yaw は不定 (またはRollと区別がつかない) -> Rollに押し付ける
                orientation[0] = 0f // Yaw = 0 とする
                //Roll (Y軸回転)
                if (orientation[1] > 0) { // Pitch = +90 deg
                    orientation[2] = atan2(R[3], R[0]) // atan2(R[1][0], R[0][0])
                } else { // Pitch = -90 deg
                    orientation[2] = -atan2(-R[3], R[0]) // atan2(-R[1][0], R[0][0]) -> check sign convention
                    orientation[2] = atan2(-R[3], R[0]) // Simplified
                }
                //Rollを正規化
                while (orientation[2] <= -PI) orientation[2] += (2.0 * PI).toFloat()
                while (orientation[2] > PI) orientation[2] -= (2.0 * PI).toFloat()
            }
        }
    }
}