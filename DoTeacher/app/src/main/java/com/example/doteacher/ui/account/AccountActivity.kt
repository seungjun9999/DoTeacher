package com.example.doteacher.ui.account

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.example.doteacher.R
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class AccountActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_account)

        if (savedInstanceState == null) {
            supportFragmentManager.beginTransaction()
                .replace(R.id.fragment_container, AccountFragment())
                .commit()
        }
    }
}