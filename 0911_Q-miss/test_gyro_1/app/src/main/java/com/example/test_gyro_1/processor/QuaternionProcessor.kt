package com.example.test_gyro_1.processor

import android.hardware.SensorEvent
import android.hardware.SensorManager
import android.util.Log
import kotlin.math.*

class QuaternionProcessor {

    // 絶対的な回転
    private var currentAbsoluteQuaternion: FloatArray? = null // [x, y, z, w]
    private val absoluteRotationMatrix = FloatArray(9)

    // 相対的な回転 (初期姿勢からの差分)
    private var initialAbsoluteQuaternion: FloatArray? = null
    private val relativeQuaternion = FloatArray(4) // [x, y, z, w]
    private val relativeRotationMatrix = FloatArray(9)
    private val relativeOrientationAnglesRad = FloatArray(3)
    private val relativeOrientationAnglesDeg = FloatArray(3)

    // 初期化完了フラグ
    private var isInitialized = false

    init {
        // 初期状態としてゼロ回転を設定
        relativeQuaternion[3] = 1.0f // 単位クォータニオン (w=1, x=y=z=0)
        // 回転行列も単位行列で初期化しておくのが望ましい
        setIdentityMatrix(absoluteRotationMatrix)
        setIdentityMatrix(relativeRotationMatrix)
        // relativeOrientationAnglesDeg はゼロフィル (initでデフォルト0f)
    }

    /**
     * ワールド座標系への変換に使用する絶対回転行列を取得
     */
    fun getRotationMatrix(): FloatArray? {
        // isInitialized フラグも考慮するか、currentAbsoluteQuaternion の null チェックで十分か
        return if (currentAbsoluteQuaternion != null) {
            absoluteRotationMatrix.clone()
        } else {
            null
        }
    }

    /**
     * 初期姿勢からの相対的なオイラー角 [Yaw, Pitch, Roll] を取得 (度)
     */
    fun getEulerAnglesDegrees(): FloatArray {
        return relativeOrientationAnglesDeg.clone()
    }

    /**
     * このプロセッサが初期化され、相対角度を計算できる状態か
     */
    fun isInitialized(): Boolean {
        return isInitialized
    }

    fun processRotationVectorEvent(event: SensorEvent) {
        if (event.values.size >= 4) {
            try {
                // 1. 現在の絶対クォータニオンを取得・正規化
                val currentRawQuaternion = FloatArray(4)
                SensorManager.getQuaternionFromVector(currentRawQuaternion, event.values)
                currentAbsoluteQuaternion = normalizeQuaternion(currentRawQuaternion)

                // nullチェックを追加 (normalizeQuaternion が null を返す可能性は低いが念のため)
                currentAbsoluteQuaternion?.let { currentAbsQ ->

                    // 2. 加速度変換用の絶対回転行列を計算・保持
                    // ★★★ SensorManager の代わりにクラス内の関数を使用 ★★★
                    getRotationMatrixFromQuaternion(currentAbsQ, absoluteRotationMatrix)
                    // ★★★ 修正ここまで ★★★

                    // 3. 初期クォータニオンを設定 (初回のみ)
                    if (initialAbsoluteQuaternion == null) {
                        initialAbsoluteQuaternion = currentAbsQ.clone()
                        isInitialized = true
                        Log.i("QuaternionProcessor", "Initial quaternion set.")
                        // 初回は相対回転がゼロになるはずなので、相対状態も更新
                        relativeQuaternion.fill(0f); relativeQuaternion[3] = 1.0f
                        setIdentityMatrix(relativeRotationMatrix)
                        relativeOrientationAnglesDeg.fill(0f)
                    }

                    // 4. 相対クォータニオンを計算 (Q_relative = Q_current * Q_initial_inverse)
                    initialAbsoluteQuaternion?.let { initialAbsQ -> // initialAbsQ も non-null を確認
                        val initialInverse = inverseQuaternion(initialAbsQ)
                        multiplyQuaternions(relativeQuaternion, currentAbsQ, initialInverse)
                        normalizeQuaternion(relativeQuaternion, relativeQuaternion)

                        // 5. 相対クォータニオンから相対回転行列を計算
                        // ★★★ SensorManager の代わりにクラス内の関数を使用 ★★★
                        getRotationMatrixFromQuaternion(relativeQuaternion, relativeRotationMatrix)
                        // ★★★ 修正ここまで ★★★

                        // 6. 相対回転行列から相対オイラー角を計算
                        SensorManager.getOrientation(relativeRotationMatrix, relativeOrientationAnglesRad)

                        // 7. ラジアンから度数に変換し、正規化して保持
                        relativeOrientationAnglesDeg[0] = Math.toDegrees(relativeOrientationAnglesRad[0].toDouble()).toFloat()
                        relativeOrientationAnglesDeg[1] = Math.toDegrees(relativeOrientationAnglesRad[1].toDouble()).toFloat()
                        relativeOrientationAnglesDeg[2] = Math.toDegrees(relativeOrientationAnglesRad[2].toDouble()).toFloat()
                        relativeOrientationAnglesDeg[0] = normalizeAngle(relativeOrientationAnglesDeg[0])
                        relativeOrientationAnglesDeg[1] = normalizeAngle(relativeOrientationAnglesDeg[1])
                        relativeOrientationAnglesDeg[2] = normalizeAngle(relativeOrientationAnglesDeg[2])

                    } ?: run {
                        // initialAbsoluteQuaternion が null の場合 (通常は起こらないはずだが)
                        resetRelativeState()
                    }

                } ?: run {
                    // currentAbsoluteQuaternion が null になった場合 (normalize に失敗した場合など)
                    Log.w("QuaternionProcessor", "currentAbsoluteQuaternion became null after normalization attempt.")
                    resetInternalState() // 状態をリセット
                }

            } catch (e: IllegalArgumentException) {
                Log.e("QuaternionProcessor", "Error processing rotation vector: ${e.message}")
                resetInternalState()
            } catch (e: Exception) {
                Log.e("QuaternionProcessor", "Unexpected error processing rotation vector", e)
                resetInternalState()
            }
        } else {
            Log.w("QuaternionProcessor", "Received rotation vector event with unexpected size: ${event.values.size}")
            resetInternalState()
        }
    }

    /**
     * プロセッサの状態をリセットする
     */
    fun reset() {
        Log.d("QuaternionProcessor", "Resetting QuaternionProcessor state.")
        resetInternalState()
        isInitialized = false
        initialAbsoluteQuaternion = null
    }

    /**
     * クォータニオンおよび角度の内部状態をゼロ/初期値にリセット
     */
    private fun resetInternalState() {
        currentAbsoluteQuaternion = null
        setIdentityMatrix(absoluteRotationMatrix) // 単位行列でリセット
        resetRelativeState() // 相対状態もリセット
    }

    /**
     * 相対回転に関する状態のみをリセット
     */
    private fun resetRelativeState() {
        relativeQuaternion.fill(0f)
        relativeQuaternion[3] = 1.0f // 単位クォータニオン
        setIdentityMatrix(relativeRotationMatrix) // 単位行列
        relativeOrientationAnglesRad.fill(0f)
        relativeOrientationAnglesDeg.fill(0f)
    }

    // --- クォータニオン ヘルパー関数 ---

    private fun normalizeQuaternion(q: FloatArray, out: FloatArray = FloatArray(4)): FloatArray? {
        val norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        return if (norm > 1e-9f) { // ゼロ除算を避ける閾値
            val invNorm = 1.0f / norm
            out[0] = q[0] * invNorm
            out[1] = q[1] * invNorm
            out[2] = q[2] * invNorm
            out[3] = q[3] * invNorm
            out
        } else {
            Log.w("QuaternionProcessor", "Quaternion norm is too small to normalize: ${q.joinToString()}")
            // ゼロクォータニオンに近い場合は null を返すか、単位クォータニオンにするか
            // null を返して呼び出し元で処理する方が安全かもしれない
            null
            // または単位クォータニオンにフォールバック
            // out[0] = 0f; out[1] = 0f; out[2] = 0f; out[3] = 1f
            // out
        }
    }

    private fun inverseQuaternion(q: FloatArray): FloatArray {
        // 正規化されている前提 q^-1 = [-x, -y, -z, w]
        return floatArrayOf(-q[0], -q[1], -q[2], q[3])
    }

    private fun multiplyQuaternions(q_result: FloatArray, q1: FloatArray, q2: FloatArray) {
        val x1 = q1[0]; val y1 = q1[1]; val z1 = q1[2]; val w1 = q1[3]
        val x2 = q2[0]; val y2 = q2[1]; val z2 = q2[2]; val w2 = q2[3]
        q_result[0] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        q_result[1] = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        q_result[2] = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        q_result[3] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    }

    // --- オイラー角正規化 ---
    private fun normalizeAngle(angle: Float): Float {
        var normalized = angle % 360.0f //剰余の計算結果は入力の符号に依存する場合があるので注意
        // 常に (-180, 180] の範囲にする
        while (normalized <= -180f) normalized += 360f
        while (normalized > 180f) normalized -= 360f
        return normalized
    }

    // --- 回転行列ヘルパー ---

    /**
     * クラス内の独自実装: クォータニオンから回転行列を計算
     */
    private fun getRotationMatrixFromQuaternion(q: FloatArray, matrix: FloatArray) {
        if (q.size < 4 || matrix.size < 9) return

        // クォータニオンを正規化 (入力が正規化済みでも念のため)
        val norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        if (norm < 1e-9f) {
            Log.w("getRotationMatrixFromQuaternion", "Input quaternion norm is too small.")
            setIdentityMatrix(matrix) // 不正な入力の場合は単位行列を設定
            return
        }
        val invNorm = 1.0f / norm
        val x = q[0] * invNorm
        val y = q[1] * invNorm
        val z = q[2] * invNorm
        val w = q[3] * invNorm

        // 回転行列の要素を計算 (前のコードと同じロジック)
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

    /**
     * 3x3 回転行列を単位行列に設定
     */
    private fun setIdentityMatrix(matrix: FloatArray) {
        if (matrix.size < 9) return
        matrix.fill(0f)
        matrix[0] = 1f; matrix[4] = 1f; matrix[8] = 1f
    }
}