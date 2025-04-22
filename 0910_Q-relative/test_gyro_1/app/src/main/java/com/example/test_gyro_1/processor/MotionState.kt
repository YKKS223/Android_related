package com.example.test_gyro_1.processor

import android.graphics.PointF
import com.example.test_gyro_1.gps.LocationData
import android.R

data class MotionState(
    val position: FloatArray = FloatArray(3), //[x, y, z]
    val velocity: FloatArray = FloatArray(3), //[vx, vy, vz]
    val worldAccel: FloatArray = FloatArray(3), //ワールド加速度 [ax, ay, az] (デバッグ用)
    val angleYaw: Float = 0f,   //Z軸周り (方位角) - ★相対角度になる★
    val anglePitch: Float = 0f, //X軸周り (傾き 前後) - ★相対角度になる★
    val angleRoll: Float = 0f,  //Y軸周り (傾き 左右) - ★相対角度になる★
    val isStationary: Boolean = false,
    val isHighSpeedMode: Boolean = false,
    val speedMps: Float = 0f,
    val speedKmh: Float = 0f,
    val totalDistance: Float = 0f, //原点からの直線距離(変位)
    val accumulatedDistance: Float = 0f, //移動総距離(道のり)
    val currentPoint: PointF? = null,
    val pathHistory: List<PointF> = emptyList(), //軌跡データ(PointFのリスト)
    val statusText: String = "初期化中",
    val statusColor: Int = R.color.darker_gray,
    val latestGpsData: LocationData? = null
) {
    // equals, hashCode は自動生成または手動で適切に定義されているはずなので省略 (変更なし)
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as MotionState

        if (!position.contentEquals(other.position)) return false
        if (!velocity.contentEquals(other.velocity)) return false
        if (!worldAccel.contentEquals(other.worldAccel)) return false
        if (angleYaw != other.angleYaw) return false
        if (anglePitch != other.anglePitch) return false
        if (angleRoll != other.angleRoll) return false
        if (isStationary != other.isStationary) return false
        if (isHighSpeedMode != other.isHighSpeedMode) return false
        if (speedMps != other.speedMps) return false
        if (speedKmh != other.speedKmh) return false
        if (totalDistance != other.totalDistance) return false
        if (accumulatedDistance != other.accumulatedDistance) return false
        if (currentPoint != other.currentPoint) return false
        if (pathHistory != other.pathHistory) return false
        if (statusText != other.statusText) return false
        if (statusColor != other.statusColor) return false
        if (latestGpsData != other.latestGpsData) return false

        return true
    }

    override fun hashCode(): Int {
        var result = position.contentHashCode()
        result = 31 * result + velocity.contentHashCode()
        result = 31 * result + worldAccel.contentHashCode()
        result = 31 * result + angleYaw.hashCode()
        result = 31 * result + anglePitch.hashCode()
        result = 31 * result + angleRoll.hashCode()
        result = 31 * result + isStationary.hashCode()
        result = 31 * result + isHighSpeedMode.hashCode()
        result = 31 * result + speedMps.hashCode()
        result = 31 * result + speedKmh.hashCode()
        result = 31 * result + totalDistance.hashCode()
        result = 31 * result + accumulatedDistance.hashCode()
        result = 31 * result + (currentPoint?.hashCode() ?: 0)
        result = 31 * result + pathHistory.hashCode()
        result = 31 * result + statusText.hashCode()
        result = 31 * result + statusColor
        result = 31 * result + (latestGpsData?.hashCode() ?: 0)
        return result
    }

    companion object {
        fun initial(): MotionState = MotionState()
    }
}