package com.example.test_gyro_1.filter

import android.util.Log
import com.example.test_gyro_1.gps.LocationData
import kotlin.math.max
import com.example.test_gyro_1.filter.SimpleMatrix
import kotlin.math.sqrt

/**
 * 位置と速度を推定する拡張カルマンフィルタ
 * 状態ベクトル x = [px, py, pz, vx, vy, vz]^T (6x1)
 */
class ExtendedKalmanFilter {

    //EKF状態
    var x: SimpleMatrix //状態ベクトル [px, py, pz, vx, vy, vz] (6x1)
    var P: SimpleMatrix //状態推定誤差共分散行列 (6x6)

    //EKFパラメータ(チューニングが必要)
    //プロセスノイズ共分散(加速度の不確かさ)
    //対角成分:sigma_a^2(m/s^2)^2
    private val accelNoiseStdDev = 0.5f //加速度ノイズの標準偏差 (m/s^2) - 要調整
    //観測ノイズ共分散(静止状態での速度観測の不確かさ)
    //対角成分:sigma_v^2 (m/s)^2
    private val stationaryVelNoiseStdDev = 0.01f //静止時速度ノイズの標準偏差 (m/s) - 要調整

    //private var lastEkfPositionForShortDistanceCheck: FloatArray? = null //最後にGPS更新した時のEKF位置
    //最後にこのフィルタで更新に使用した有効なGPSデータを保持する変数
    private var previousGpsDataUsedForUpdate: LocationData? = null
    private val SHORT_DISTANCE_THRESHOLD = 1.0f //m(これ未満のGPS移動ではGPS更新を抑制) bf2


    //固定の行列
    private val I = SimpleMatrix.identity(6) //単位行列 (6x6)
    //静止状態観測モデル H_stationary = [ 0 0 0 I(3) ] (3x6)
    private val H_stationary = SimpleMatrix(3, 6) { r, c -> if (c == r + 3) 1.0f else 0.0f }
    //静止状態観測ノイズ共分散 R_stationary (3x3)
    private val R_stationary = SimpleMatrix(3, 3) { r, c ->
        if (r == c) stationaryVelNoiseStdDev * stationaryVelNoiseStdDev else 0.0f
    }

    //計算用一時変数
    private val F = SimpleMatrix.identity(6) //状態遷移行列 (予測時に更新)
    private val Q_base = SimpleMatrix(6, 6) //基本プロセスノイズ (予測時にスケーリング)

    //GPS観測用パラメータ
    private val H_gps_pos_3d = SimpleMatrix(3, 6) { r, c -> if (c == r) 1.0f else 0.0f } // 3次元位置観測 (XYZ)
    private val H_gps_pos_2d = SimpleMatrix(2, 6) { r, c -> if (c == r) 1.0f else 0.0f } // 2次元(XY)位置観測

    //GPS観測ノイズ調整用パラメータ
    //XY方向の信頼度を高めるための係数 (1.0より小さくすると信頼度UP)
    private val GPS_XY_NOISE_FACTOR = 0.4f //例: GPS精度を半分として扱う (要調整！)
    //Z方向(高度)の信頼度を低く保つための係数 (1.0より大きくすると信頼度DOWN)
    private val GPS_Z_NOISE_FACTOR = 3.0f  //例: GPS精度を3倍として扱う (要調整！)
    //GPS精度が非常に良くても、過信しないための最低標準偏差 (m)
    private val GPS_MIN_STD_DEV = 1.0f //bf2
    private val GPS_XY_NOISE_FACTOR_HIGH_SPEED = 0.1f // 高速時のXY信頼度をさらにUP (要調整！)

    /**
     * EKF 更新ステップ (GPS位置観測)
     * @param gpsData GPSマネージャーから受け取ったデータ
     * @param isHighSpeedMode MainActivity から渡される高速移動モードフラグ
     */
    fun updateGpsPosition(gpsData: LocationData, isHighSpeedMode: Boolean) {
        //短距離移動チェック
        val previousGps = previousGpsDataUsedForUpdate
        if (previousGps != null && gpsData.isLocalValid && previousGps.isLocalValid &&
            gpsData.localX != null && gpsData.localY != null &&
            previousGps.localX != null && previousGps.localY != null) {

            val dxGps = gpsData.localX - previousGps.localX
            val dyGps = gpsData.localY - previousGps.localY
            //Z座標(高度)も比較に含めるかは要検討 (GPS高度精度は低い傾向があるためXY平面で比較するのが一般的)
            //val dzGps = (gpsData.localZ ?: 0f) - (previousGps.localZ ?: 0f)
            val gpsDistanceMoved = sqrt(dxGps * dxGps + dyGps * dyGps /* + dzGps * dzGps */) //XY平面での距離

            if (gpsDistanceMoved < SHORT_DISTANCE_THRESHOLD) {
                //Log.d("EKF_Update_GPS", "Skipping GPS update: Short GPS distance moved since last update (%.2fm < %.1fm)".format(gpsDistanceMoved, SHORT_DISTANCE_THRESHOLD))
                //短距離移動なので、GPS更新をスキップする。
                //この場合、previousGpsDataUsedForUpdate は更新しない。
                return //スキップ
            } else {
            // Log.d("EKF_Update_GPS", "Proceeding with GPS update: GPS distance moved %.2fm >= %.1fm".format(gpsDistanceMoved, SHORT_DISTANCE_THRESHOLD))
            // 閾値を超えたので、この後GPS更新を実行し、previousGpsDataUsedForUpdate も更新する
            }
        } else {
        //初回のGPS更新、またはリセット後の最初の有効なGPSデータ
        //Log.d("EKF_Update_GPS", "Proceeding with first valid GPS update or update after reset.")
        //この後GPS更新を実行し、previousGpsDataUsedForUpdate を設定する
        }

        //既存のGPS有効性チェック
        val maxAccuracy = 50.0f
        if (!gpsData.isLocalValid ||
            gpsData.localX == null || gpsData.localY == null ||
            gpsData.accuracy == null || gpsData.accuracy <= 0f || gpsData.accuracy > maxAccuracy) {
            val reason = when {
                !gpsData.isLocalValid -> "isLocalValid is false"
                gpsData.localX == null -> "localX is null"
                gpsData.localY == null -> "localY is null"
                gpsData.accuracy == null -> "accuracy is null"
                gpsData.accuracy <= 0f -> "accuracy <= 0"
                gpsData.accuracy > maxAccuracy -> "accuracy > maxAccuracy (${gpsData.accuracy}m)"
                else -> "Unknown reason" // 通常ここには来ないはず
            }
            Log.w("EKF_Update_GPS", "Skipping GPS update: $reason")
            return
        }


        //(既存の R_gps 計算、2D/3D分岐、カルマンゲイン計算、状態更新ロジックはそのまま)
        val gpsAccuracyStdDev = max(gpsData.accuracy, GPS_MIN_STD_DEV)
        val useZ = gpsData.localZ != null
        val H: SimpleMatrix
        val z: SimpleMatrix
        val R_gps: SimpleMatrix
        //(H, z, R_gps の設定ロジック)

        //高速モードに応じて XY ノイズ係数を切り替え
        val currentXYNoiseFactor = if (isHighSpeedMode) {
            GPS_XY_NOISE_FACTOR_HIGH_SPEED // 高速モードなら、より小さい係数を使う
        } else {
            GPS_XY_NOISE_FACTOR // 通常モード
        }

        if (useZ) {
            // 3D 更新 (XYZ)
            H = H_gps_pos_3d
            z = SimpleMatrix(3, 1); z[0,0]=gpsData.localX!!; z[1,0]=gpsData.localY!!; z[2,0]=gpsData.localZ!!
            // ★★★ currentXYNoiseFactor を使用 ★★★
            val stdDevXY = gpsAccuracyStdDev * currentXYNoiseFactor
            val stdDevZ = gpsAccuracyStdDev * GPS_Z_NOISE_FACTOR // Z はモードに関わらず同じ
            R_gps = SimpleMatrix(3, 3); R_gps[0,0]=stdDevXY*stdDevXY; R_gps[1,1]=stdDevXY*stdDevXY; R_gps[2,2]=stdDevZ*stdDevZ
            Log.d("EKF_Update_GPS", "Performing 3D GPS update (HighSpeed: $isHighSpeedMode). Acc: ${gpsData.accuracy}m, R_XY: %.2f, R_Z: %.2f".format(R_gps[0,0], R_gps[2,2]))
        } else {
            // 2D 更新 (XY)
            H = H_gps_pos_2d
            z = SimpleMatrix(2, 1); z[0,0]= gpsData.localX!!; z[1,0]= gpsData.localY!!
            // ★★★ currentXYNoiseFactor を使用 ★★★
            val stdDevXY = gpsAccuracyStdDev * currentXYNoiseFactor
            R_gps = SimpleMatrix(2, 2); R_gps[0,0]=stdDevXY*stdDevXY; R_gps[1,1]=stdDevXY*stdDevXY
            Log.d("EKF_Update_GPS", "Performing 2D GPS update (HighSpeed: $isHighSpeedMode). Acc: ${gpsData.accuracy}m, R_XY: %.2f".format(R_gps[0,0]))
        }

        // ... (y, P_HT, S, S_inv, K の計算ロジック)
        val y = z - (H * x)
        val P_HT = P * H.transpose()
        val S = H * P_HT + R_gps
        val S_inv: SimpleMatrix? = if (useZ) S.inverse3x3() else S.inverse2x2()
        if (S_inv == null) {
            Log.e("EKF_Update_GPS", "Failed to invert S matrix...")
            return
        }
        val K = P_HT * S_inv

        //状態更新
        x += K * y
        P = (I - K * H) * P

        Log.i("EKF_Update_GPS", "GPS Position Update Applied (${if(useZ) "3D" else "2D"}). Accuracy: ${gpsData.accuracy}m")

        //最後にGPS更新した位置を記録
        //GPS更新が実際に行われた場合にのみ、現在のEKF位置を記録する
        previousGpsDataUsedForUpdate = gpsData.copy()
    }

    /**
     * EKFの状態をリセットする
     */
    fun reset() {
        x = SimpleMatrix(6, 1)
        P = SimpleMatrix.identity(6) * 1.0f
        //リセット時に履歴もクリア
        previousGpsDataUsedForUpdate = null

        Log.d("EKF", "EKF Reset.")
    }


    init {
        //初期状態設定
        x = SimpleMatrix(6, 1) // 位置(0,0,0), 速度(0,0,0)
        P = SimpleMatrix.identity(6) * 1.0f // 初期共分散 (大きめに設定)

        //基本プロセスノイズ Q_base の設定
        //Q = G * Q_a * G^T に基づく設計
        //Q_a = diag(sigma_a^2, sigma_a^2, sigma_a^2)
        val q_noise = accelNoiseStdDev * accelNoiseStdDev
        //G = [ 0.5*dt^2 * I(3) ]
        //[   dt * I(3)     ]
        //簡略化のため、位置と速度のノイズを直接設定するアプローチもある
        //ここでは、各状態変数のノイズを対角成分で設定する
        val posInitNoise = 0.01f // 位置の初期プロセスノイズ分散 (m^2) - 要調整
        val velInitNoise = q_noise   // 速度の初期プロセスノイズ分散 ((m/s)^2) - 要調整
        Q_base[0, 0] = posInitNoise; Q_base[1, 1] = posInitNoise; Q_base[2, 2] = posInitNoise
        Q_base[3, 3] = velInitNoise; Q_base[4, 4] = velInitNoise; Q_base[5, 5] = velInitNoise

        Log.d("EKF", "EKF Initialized. Initial State:\n$x \nInitial Covariance P:\n$P")
    }

    /**
     * EKF 予測ステップ
     * @param dt 時間ステップ (秒)
     * @param worldAccel ワールド座標系の加速度 [ax, ay, az] (m/s^2)
     */


    fun predict(dt: Float, worldAccel: FloatArray) {
        if (dt <= 0) return

        //1. 状態遷移行列 F の更新
        //F = [ I(3) dt*I(3) ]
        //[ 0(3)   I(3)   ]
        F[0, 3] = dt; F[1, 4] = dt; F[2, 5] = dt
        //対角要素は常に1 (initで設定済み、変更なし)

        //2. プロセスノイズ Q の計算 (dtに依存させる)
        //簡単のため Q = Q_base * dt とする（より正確には運動モデルに合わせるべき）
        //運動モデルに基づくQ:
        //G = [ 0.5*dt^2; dt ] (各軸独立と仮定)
        //Q = G * sigma_a^2 * G^T
        val dt2 = dt * dt
        val dt3_over_2 = 0.5f * dt * dt2 // dt^3 / 2
        val dt4_over_4 = 0.25f * dt2 * dt2 // dt^4 / 4
        val q_noise = accelNoiseStdDev * accelNoiseStdDev

        val Q = SimpleMatrix(6, 6)
        // 対角ブロック
        Q[0, 0] = dt4_over_4 * q_noise; Q[1, 1] = Q[0, 0]; Q[2, 2] = Q[0, 0] // Pos-Pos
        Q[3, 3] = dt2 * q_noise;        Q[4, 4] = Q[3, 3]; Q[5, 5] = Q[3, 3] // Vel-Vel
        // 非対角ブロック
        Q[0, 3] = dt3_over_2 * q_noise; Q[3, 0] = Q[0, 3]
        Q[1, 4] = Q[0, 3];              Q[4, 1] = Q[0, 3]
        Q[2, 5] = Q[0, 3];              Q[5, 2] = Q[0, 3]


        //3. 状態予測 x_pred = f(x, u)
        //f(x) = F * x + G * a * dt^2 / 2 (位置) + G * a * dt (速度)
        val px = x[0, 0]; val py = x[1, 0]; val pz = x[2, 0]
        val vx = x[3, 0]; val vy = x[4, 0]; val vz = x[5, 0]
        val ax = worldAccel[0]; val ay = worldAccel[1]; val az = worldAccel[2]

        val x_pred = SimpleMatrix(6, 1)
        x_pred[0, 0] = px + vx * dt + 0.5f * ax * dt2
        x_pred[1, 0] = py + vy * dt + 0.5f * ay * dt2
        x_pred[2, 0] = pz + vz * dt + 0.5f * az * dt2
        x_pred[3, 0] = vx + ax * dt
        x_pred[4, 0] = vy + ay * dt
        x_pred[5, 0] = vz + az * dt

        //4. 共分散予測 P_pred = F * P * F^T + Q
        val P_pred = F * P * F.transpose() + Q

        //予測結果を更新
        x = x_pred
        P = P_pred

        //Log.d("EKF_Predict", "dt: $dt, worldAccel: ${worldAccel.joinToString()}, Predicted State:\n$x \nPredicted Covariance P:\n$P")
    }

    /**
     * EKF 更新ステップ (静止状態観測)
     * 観測値 z = [0, 0, 0]^T (速度) を使用
     */
    fun updateStationary() {
        //観測値 z = [0, 0, 0]^T (速度)
        val z = SimpleMatrix(3, 1) // ゼロベクトル

        //カルマンゲイン計算
        //y = z - h(x) = z - H_stationary * x (観測残差)
        val y = z - (H_stationary * x)

        //S = H * P * H^T + R (残差共分散)
        val P_HT = P * H_stationary.transpose() // P * H^T は複数回使うので計算しておく
        val S = H_stationary * P_HT + R_stationary

        //Sの逆行列 S_inv
        val S_inv = S.inverse3x3()
        if (S_inv == null) {
            Log.e("EKF", "Failed to invert S matrix in stationary update. S:\n$S")
            // S が逆行列を持たない場合、更新をスキップするか、P をリセットするなどの対策が必要
            // ここではスキップ
            return
        }

        //K = P * H^T * S^-1 (カルマンゲイン)
        val K = P_HT * S_inv

        //状態と共分散の更新
        //x_new = x + K * y
        x += K * y

        //P_new = (I - K * H) * P (Joseph formの方が安定する場合がある: (I-KH)P(I-KH)^T + KRK^T)
        P = (I - K * H_stationary) * P

        //Pが対称性を失う可能性があるので、強制的に対称にする (オプション)
        //P = (P + P.transpose()) * 0.5f

        //速度を強制的にゼロにする（オプション、静止状態の確信度が高い場合）
        //x[3, 0] = 0f; x[4, 0] = 0f; x[5, 0] = 0f

        Log.d("EKF_Update", "Stationary Update Applied. Updated State:\n$x \nUpdated Covariance P:\n$P")
    }

    //現在の状態を取得するメソッド (必要に応じて)
    fun getState(): FloatArray = x.data.copyOf()
    fun getPosition(): FloatArray = floatArrayOf(x[0, 0], x[1, 0], x[2, 0])
    fun getVelocity(): FloatArray = floatArrayOf(x[3, 0], x[4, 0], x[5, 0])

    //(オプション) GPS速度更新メソッド (今回は使わない)
    //fun updateGpsVelocity(gpsData: LocationData) { ... }

}