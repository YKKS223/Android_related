package com.example.test_gyro_1.processor

import com.example.test_gyro_1.gps.LocationData

//UIに表示するための状態を保持するデータクラス
data class MotionState(
    val position: FloatArray = FloatArray(3), //[x, y, z]
    val velocity: FloatArray = FloatArray(3), //[vx, vy, vz]
    val worldAccel: FloatArray = FloatArray(3), //ワールド加速度[ax, ay, az](デバッグ用)
    val angleX: Float = 0f, //ジャイロ積分角度X
    val angleY: Float = 0f, //ジャイロ積分角度Y
    val angleZ: Float = 0f, //ジャイロ積分角度Z
    val isStationary: Boolean = false,
    val isHighSpeedMode: Boolean = false,
    val speedMps: Float = 0f, //速度(m/s)
    val speedKmh: Float = 0f, //時速(km/h)
    val totalDistance: Float = 0f, //原点からの変位
    val accumulatedDistance: Float = 0f, //移動総距離
    val statusText: String = "初期化中", //状態表示テキスト
    val statusColor: Int = android.R.color.darker_gray, //状態表示テキストの色リソースID
    val latestGpsData: LocationData? = null //最新のGPSデータ(オプション表示用)
) {
    //FloatArrayの内容比較のためのequalsとhashCode(必要に応じて)
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as MotionState

        if (!position.contentEquals(other.position)) return false
        if (!velocity.contentEquals(other.velocity)) return false
        if (!worldAccel.contentEquals(other.worldAccel)) return false
        if (angleX != other.angleX) return false
        if (angleY != other.angleY) return false
        if (angleZ != other.angleZ) return false
        if (isStationary != other.isStationary) return false
        if (isHighSpeedMode != other.isHighSpeedMode) return false
        if (speedMps != other.speedMps) return false
        if (speedKmh != other.speedKmh) return false
        if (totalDistance != other.totalDistance) return false
        if (accumulatedDistance != other.accumulatedDistance) return false
        if (statusText != other.statusText) return false
        if (statusColor != other.statusColor) return false
        if (latestGpsData != other.latestGpsData) return false

        return true
    }

    override fun hashCode(): Int {
        var result = position.contentHashCode()
        result = 31 * result + velocity.contentHashCode()
        result = 31 * result + worldAccel.contentHashCode()
        result = 31 * result + angleX.hashCode()
        result = 31 * result + angleY.hashCode()
        result = 31 * result + angleZ.hashCode()
        result = 31 * result + isStationary.hashCode()
        result = 31 * result + isHighSpeedMode.hashCode()
        result = 31 * result + speedMps.hashCode()
        result = 31 * result + speedKmh.hashCode()
        result = 31 * result + totalDistance.hashCode()
        result = 31 * result + accumulatedDistance.hashCode()
        result = 31 * result + statusText.hashCode()
        result = 31 * result + statusColor
        result = 31 * result + (latestGpsData?.hashCode() ?: 0)
        return result
    }

    companion object {
        //初期状態を返す
        fun initial(): MotionState = MotionState()
    }
}