package com.example.test_gyro_1.gps

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.os.Looper
import android.util.Log
import com.google.android.gms.location.*
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.callbackFlow
import kotlin.math.*
import android.os.Build

class GpsManager(context: Context) {

    private val fusedLocationClient: FusedLocationProviderClient =
        LocationServices.getFusedLocationProviderClient(context)

    private val _locationDataFlow = MutableStateFlow<LocationData?>(null)
    val locationDataFlow: StateFlow<LocationData?> = _locationDataFlow

    //ローカル座標系の原点(最初の有効なGPS位置)
    private var originLocation: Location? = null
    private var isOriginSet = false

    private val locationRequest = LocationRequest.create().apply {
        interval = 1000 //更新間隔(ms)-必要に応じて調整
        fastestInterval = 500 //最速更新間隔(ms)
        priority = LocationRequest.PRIORITY_HIGH_ACCURACY //高精度
    }

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(locationResult: LocationResult) {
            locationResult.lastLocation?.let { location ->
                Log.d("GpsManager", "GPS Update: Lat=${location.latitude}, Lon=${location.longitude}, Alt=${location.altitude}, Speed=${location.speed}, Acc=${location.accuracy}")

                //ローカル座標変換(簡易版)
                if (!isOriginSet && location.hasAccuracy() && location.accuracy < 50.0f) { // 精度が良い最初の位置を原点とする
                    originLocation = location
                    isOriginSet = true
                    Log.i("GpsManager", "GPS Origin Set: Lat=${originLocation!!.latitude}, Lon=${originLocation!!.longitude}")
                }

                var localX: Float? = null
                var localY: Float? = null
                var localZ: Float? = null
                var localVx: Float? = null
                var localVy: Float? = null
                var isLocalValid = false

                if (isOriginSet) {
                    //緯度経度からローカルXYへの変換(簡易的な平面近似 - 短距離向け)
                    val results = FloatArray(1)
                    //Y軸(南北方向、北がプラス)の計算
                    Location.distanceBetween(originLocation!!.latitude, originLocation!!.longitude, location.latitude, originLocation!!.longitude, results)
                    localY = if (location.latitude >= originLocation!!.latitude) results[0] else -results[0]

                    //X軸(東西方向、東がプラス)の計算
                    Location.distanceBetween(originLocation!!.latitude, originLocation!!.longitude, originLocation!!.latitude, location.longitude, results)
                    localX = if (location.longitude >= originLocation!!.longitude) results[0] else -results[0]

                    //Z軸(高度)
                    localZ = if (originLocation!!.hasAltitude() && location.hasAltitude()) {
                        (location.altitude - originLocation!!.altitude).toFloat()
                    } else {
                        null //高度情報がない場合はZは使えない
                    }

                    //速度(GPS速度は水平速度の大きさのみ)
                    //注意:この簡易版ではGPS速度をVx, Vyに分解できない
                    //より高度な実装では進行方向(bearing)を使うか、
                    //あるいはEKF更新では速度の大きさのみを使うなどの工夫が必要
                    //ここでは速度は一旦nullにしておく
                    localVx = null
                    localVy = null

                    //精度が良い場合のみ有効とする(例: 水平精度 20m 以下)
                    isLocalValid = location.hasAccuracy() && location.accuracy < 20.0f
                    Log.d("GpsManager", "Local Coords: X=$localX, Y=$localY, Z=$localZ, Valid=$isLocalValid")
                }

                val newData = LocationData(
                    timestamp = System.currentTimeMillis(),
                    latitude = location.latitude,
                    longitude = location.longitude,
                    altitude = if (location.hasAltitude()) location.altitude else null,
                    speed = if (location.hasSpeed()) location.speed else null,
                    accuracy = if (location.hasAccuracy()) location.accuracy else null,

                    speedAccuracy = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        //APIレベルが26以上の場合のみ速度精度関連メソッドを呼び出す
                        if (location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond else null
                    } else {
                        //APIレベルが26未満の場合は速度精度は利用できない
                        null
                    },

                    localX = localX,
                    localY = localY,
                    localZ = localZ,
                    localVx = localVx,
                    localVy = localVy,
                    isLocalValid = isLocalValid
                )
                _locationDataFlow.value = newData
            }
        }
    }

    @SuppressLint("MissingPermission") //権限チェックは呼び出し元(Activity)で行う前提
    fun startLocationUpdates() {
        Log.i("GpsManager", "Starting location updates")
        try {
            fusedLocationClient.requestLocationUpdates(
                locationRequest,
                locationCallback,
                Looper.getMainLooper() /* UIスレッドでコールバックを受け取る */
            )
            //原点をリセット
            isOriginSet = false
            originLocation = null
        } catch (e: SecurityException) {
            Log.e("GpsManager", "Location permission not granted.", e)
            //必要であればエラー状態をFlowで通知するなど
        } catch (e: Exception) {
            Log.e("GpsManager", "Error starting location updates.", e)
        }
    }

    fun stopLocationUpdates() {
        Log.i("GpsManager", "Stopping location updates")
        fusedLocationClient.removeLocationUpdates(locationCallback)
        _locationDataFlow.value = null //停止したらデータをクリア
    }

    //必要に応じて原点をリセットするメソッド
    fun resetOrigin() {
        isOriginSet = false
        originLocation = null
        Log.i("GpsManager", "GPS Origin reset.")
    }
}