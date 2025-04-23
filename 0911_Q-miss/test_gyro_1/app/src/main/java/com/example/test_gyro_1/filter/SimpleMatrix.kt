package com.example.test_gyro_1.filter

import kotlin.math.abs
import android.util.Log
import kotlin.math.*

/**
 * 簡単な行列演算クラス (EKF用)
 */
class SimpleMatrix(val rows: Int, val cols: Int) {
    val data = FloatArray(rows * cols)

    constructor(rows: Int, cols: Int, initialData: FloatArray) : this(rows, cols) {
        if (initialData.size != rows * cols) {
            throw IllegalArgumentException("Initial data size does not match matrix dimensions")
        }
        System.arraycopy(initialData, 0, data, 0, data.size)
    }

    constructor(rows: Int, cols: Int, initFunc: (Int, Int) -> Float) : this(rows, cols) {
        for (r in 0 until rows) {
            for (c in 0 until cols) {
                set(r, c, initFunc(r, c))
            }
        }
    }

    operator fun get(row: Int, col: Int): Float {
        if (row < 0 || row >= rows || col < 0 || col >= cols) throw IndexOutOfBoundsException("Matrix index out of bounds")
        return data[row * cols + col]
    }

    operator fun set(row: Int, col: Int, value: Float) {
        if (row < 0 || row >= rows || col < 0 || col >= cols) throw IndexOutOfBoundsException("Matrix index out of bounds")
        data[row * cols + col] = value
    }

    operator fun plus(other: SimpleMatrix): SimpleMatrix {
        if (rows != other.rows || cols != other.cols) throw IllegalArgumentException("Matrix dimensions must agree for addition")
        val result = SimpleMatrix(rows, cols)
        for (i in data.indices) {
            result.data[i] = data[i] + other.data[i]
        }
        return result
    }

    operator fun minus(other: SimpleMatrix): SimpleMatrix {
        if (rows != other.rows || cols != other.cols) throw IllegalArgumentException("Matrix dimensions must agree for subtraction")
        val result = SimpleMatrix(rows, cols)
        for (i in data.indices) {
            result.data[i] = data[i] - other.data[i]
        }
        return result
    }

    operator fun times(other: SimpleMatrix): SimpleMatrix {
        if (cols != other.rows) throw IllegalArgumentException("Matrix dimensions must agree for multiplication (A.cols == B.rows)")
        val result = SimpleMatrix(rows, other.cols)
        for (r in 0 until rows) {
            for (c in 0 until other.cols) {
                var sum = 0f
                for (k in 0 until cols) {
                    sum += this[r, k] * other[k, c]
                }
                result[r, c] = sum
            }
        }
        return result
    }

    operator fun times(scalar: Float): SimpleMatrix {
        val result = SimpleMatrix(rows, cols)
        for (i in data.indices) {
            result.data[i] = data[i] * scalar
        }
        return result
    }

    fun transpose(): SimpleMatrix {
        val result = SimpleMatrix(cols, rows)
        for (r in 0 until rows) {
            for (c in 0 until cols) {
                result[c, r] = this[r, c]
            }
        }
        return result
    }

    //3x3行列の逆行列(正方行列でない場合や特異行列の場合はnullを返す)
    fun inverse3x3(): SimpleMatrix? {
        if (rows != 3 || cols != 3) return null // 3x3でない場合は計算しない

        val det = data[0] * (data[4] * data[8] - data[5] * data[7]) -
                data[1] * (data[3] * data[8] - data[5] * data[6]) +
                data[2] * (data[3] * data[7] - data[4] * data[6])

        if (abs(det) < 1e-10) return null // 特異行列（逆行列が存在しない）

        val invDet = 1.0f / det
        val result = SimpleMatrix(3, 3)

        result.data[0] = (data[4] * data[8] - data[5] * data[7]) * invDet
        result.data[1] = (data[2] * data[7] - data[1] * data[8]) * invDet
        result.data[2] = (data[1] * data[5] - data[2] * data[4]) * invDet
        result.data[3] = (data[5] * data[6] - data[3] * data[8]) * invDet
        result.data[4] = (data[0] * data[8] - data[2] * data[6]) * invDet
        result.data[5] = (data[2] * data[3] - data[0] * data[5]) * invDet
        result.data[6] = (data[3] * data[7] - data[4] * data[6]) * invDet
        result.data[7] = (data[1] * data[6] - data[0] * data[7]) * invDet
        result.data[8] = (data[0] * data[4] - data[1] * data[3]) * invDet

        return result
    }

    override fun toString(): String {
        val sb = StringBuilder()
        for (r in 0 until rows) {
            sb.append("[ ")
            for (c in 0 until cols) {
                sb.append(String.format("%.4f ", this[r, c]))
            }
            sb.append("]\n")
        }
        return sb.toString()
    }

    companion object {
        fun identity(size: Int): SimpleMatrix {
            val result = SimpleMatrix(size, size)
            for (i in 0 until size) {
                result[i, i] = 1.0f
            }
            return result
        }
    }

    //2x2行列の逆行列
    fun inverse2x2(): SimpleMatrix? {
        if (rows != 2 || cols != 2) {
            //Log.e("SimpleMatrix","inverse2x2 called on non-2x2 matrix($rows x $cols)")
            return null //2x2でない場合は計算しない
        }

        val a = data[0]; val b = data[1]
        val c = data[2]; val d = data[3]
        val det = a * d - b * c //行列式 ad - bc

        if (abs(det) < 1e-10f) { //ほぼゼロなら特異行列
            //Log.e("SimpleMatrix", "inverse2x2 failed: Determinant is close to zero ($det)")
            return null
        }

        val invDet = 1.0f / det
        val result = SimpleMatrix(2, 2)

        result.data[0] = d * invDet
        result.data[1] = -b * invDet
        result.data[2] = -c * invDet
        result.data[3] = a * invDet

        return result
    }

    /*
      4x4 行列の逆行列を計算する
      @return 逆行列 or null (計算不能な場合)
     */
    fun inverse4x4(): SimpleMatrix? {
        if (rows != 4 || cols != 4) return null

        val augmented = SimpleMatrix(4, 8) // 元の行列と単位行列を結合
        for (r in 0 until 4) {
            for (c in 0 until 4) {
                augmented[r, c] = this[r, c]
            }
            augmented[r, r + 4] = 1.0f // 単位行列部分
        }

        // ガウス・ジョルダン消去法
        for (pivotRow in 0 until 4) {
            // ピボット要素を見つける (絶対値最大)
            var maxRow = pivotRow
            for (r in pivotRow + 1 until 4) {
                if (abs(augmented[r, pivotRow]) > abs(augmented[maxRow, pivotRow])) {
                    maxRow = r
                }
            }
            // ピボット行と現在の行を入れ替え
            if (maxRow != pivotRow) {
                for (c in 0 until 8) {
                    val temp = augmented[pivotRow, c]
                    augmented[pivotRow, c] = augmented[maxRow, c]
                    augmented[maxRow, c] = temp
                }
            }

            // ピボット要素がゼロに近いかチェック
            val pivotVal = augmented[pivotRow, pivotRow]
            if (abs(pivotVal) < 1e-10f) {
                Log.e("SimpleMatrix", "Matrix is singular or near singular, cannot invert.")
                return null // 特異行列
            }

            // ピボット行を正規化 (ピボット要素を1にする)
            val invPivot = 1.0f / pivotVal
            for (c in pivotRow until 8) {
                augmented[pivotRow, c] *= invPivot
            }

            // 他の行からピボット行を引く (ピボット列の他の要素を0にする)
            for (r in 0 until 4) {
                if (r != pivotRow) {
                    val factor = augmented[r, pivotRow]
                    for (c in pivotRow until 8) {
                        augmented[r, c] -= factor * augmented[pivotRow, c]
                    }
                }
            }
        }


        // 結果の逆行列部分を取り出す
        val inverse = SimpleMatrix(4, 4)
        for (r in 0 until 4) {
            for (c in 0 until 4) {
                inverse[r, c] = augmented[r, c + 4]
            }
        }
        return inverse
    }

    fun setIdentity() {
        if (rows != cols) throw IllegalStateException("Matrix must be square to set identity")
        for (r in 0 until rows) {
            for (c in 0 until cols) {
                this[r, c] = if (r == c) 1.0f else 0.0f
            }
        }
    }

}