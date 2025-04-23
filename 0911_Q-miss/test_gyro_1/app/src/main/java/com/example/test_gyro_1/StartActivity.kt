package com.example.test_gyro_1

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.View //Viewをインポート
import android.widget.Button //Buttonをインポート
import android.widget.TextView //TextViewをインポート
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat

class StartActivity : AppCompatActivity() {

    //レイアウト要素の参照用変数を追加
    private lateinit var statusTextView: TextView
    private lateinit var startButton: Button

    private val locationPermissionRequest = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        when {
            permissions.getOrDefault(Manifest.permission.ACCESS_FINE_LOCATION, false) ||
                    permissions.getOrDefault(Manifest.permission.ACCESS_COARSE_LOCATION, false) -> {
                // いずれかの位置情報権限が許可された
                val message = if (permissions.getOrDefault(Manifest.permission.ACCESS_FINE_LOCATION, false)) {
                    "正確な位置情報の権限が許可されました。"
                } else {
                    "大まかな位置情報のみ許可されました。精度が低下する可能性があります。"
                }
                showToast(message)
                //ボタンを表示/有効化
                enableStartButton()
            }
            else -> {
                //許可されなかった
                statusTextView.text = "位置情報の許可が必要です。アプリを終了します。"
                showToast("位置情報の許可が必要です。アプリを終了します。")
                //ボタンは表示しないまま
                finish()// アクティビティを終了
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        //レイアウトファイルをセット
        setContentView(R.layout.activity_start)

        //レイアウト要素の参照を取得
        statusTextView = findViewById(R.id.textViewPermissionStatus)
        startButton = findViewById(R.id.buttonStartMain)

        //ボタンのクリックリスナーを設定
        startButton.setOnClickListener {
            navigateToMainActivity()
        }

        //権限チェックを開始
        checkAndRequestLocationPermission()
    }

    private fun checkAndRequestLocationPermission() {
        when {
            //正確な位置情報、または大まかな位置情報が既に許可されているかチェック
            ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED -> {
                //既に許可されている
                statusTextView.text = "位置情報の権限は許可されています。"
                //ボタンを表示/有効化
                enableStartButton()
                //ここでは自動遷移しない
            }
            //権限が必要な理由を表示すべきか(今回は常にリクエスト)
            //shouldShowRequestPermissionRationale(Manifest.permission.ACCESS_FINE_LOCATION) -> {
            //説明表示後にリクエスト (今回は省略)
            //requestPermission()
            //}
            else -> {
                //権限をリクエスト
                statusTextView.text = "位置情報の権限をリクエストします..."
                requestPermission()
            }
        }
    }

    private fun requestPermission() {
        //ACCESS_FINE_LOCATIONとACCESS_COARSE_LOCATIONの両方をリクエスト
        locationPermissionRequest.launch(
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
            )
        )
    }

    //計測開始ボタンを表示/有効化するメソッド
    private fun enableStartButton() {
        statusTextView.text = "準備完了です。「計測開始」ボタンを押してください。" //ステータステキスト更新
        startButton.visibility = View.VISIBLE //ボタンを表示
        startButton.isEnabled = true      //ボタンを有効化
    }

    private fun navigateToMainActivity() {
        val intent = Intent(this, MainActivity::class.java)
        startActivity(intent)
        //finish() //StartActivityは不要になるので終了
    }

    private fun showToast(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_LONG).show()
    }
}