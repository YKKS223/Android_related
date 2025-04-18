package com.example.test_gyro_1.gps

data class LocationData(
    val timestamp: Long, //システム時間 (ms)
    val latitude: Double,
    val longitude: Double,
    val altitude: Double?, //高度 (m) - 利用できない場合 null
    val speed: Float?,      //水平速度 (m/s) - 利用できない場合 null
    val accuracy: Float?,   //水平精度 (m) - 利用できない場合 null
    val speedAccuracy: Float?, //速度精度 (m/s) - 利用できない場合 null
    //ローカル座標系への変換後データ(例)
    val localX: Float?,      //ローカルX座標 (m) - 未計算または無効なら null
    val localY: Float?,      //ローカルY座標 (m) - 未計算または無効なら null
    val localZ: Float?,      //ローカルZ座標 (m) - 未計算または無効なら null
    val localVx: Float?,     //ローカルX速度 (m/s) - 未計算または無効なら null
    val localVy: Float?,     //ローカルY速度 (m/s) - 未計算または無効なら null
    // val localVz: Float?   //GPSからZ速度は通常得られない
    val isLocalValid: Boolean = false //ローカル座標データが有効か
)