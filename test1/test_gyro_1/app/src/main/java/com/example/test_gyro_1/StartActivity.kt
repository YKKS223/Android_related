package com.example.test_gyro_1

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.Button

class StartActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_start)

        val startButton: Button = findViewById(R.id.startButton)
        startButton.setOnClickListener {
            // MainActivityへの遷移
            val intent = Intent(this, MainActivity::class.java)
            startActivity(intent)
        }
    }
}