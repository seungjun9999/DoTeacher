package com.example.doteacher.ui.splash

import android.annotation.SuppressLint
import androidx.activity.viewModels
import androidx.lifecycle.lifecycleScope
import com.example.doteacher.R
import com.example.doteacher.databinding.ActivitySplashBinding
import com.example.doteacher.ui.base.BaseActivity
import com.example.doteacher.ui.login.LoginActivity
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.util.firebaseAuthCheck
import com.example.doteacher.ui.util.initGoActivity
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

@AndroidEntryPoint
class SplashActivity : BaseActivity<ActivitySplashBinding>(R.layout.activity_splash) {

    private val splashViewModel: SplashViewModel by viewModels()

    override fun init() {
        observeLoginSuccess()
        checkUser()
    }

    private fun checkUser(){
        lifecycleScope.launch {
            delay(600L)
            firebaseAuthCheck(
                complete = {
                    splashViewModel.getUserInfo(it)
                },
                cancel = {
                    initGoActivity(this@SplashActivity, LoginActivity::class.java)
                }
            )
        }
    }

    private fun observeLoginSuccess() {
        splashViewModel.loginSuccess.observe(this) {
            if(it) {
                initGoActivity(this@SplashActivity, MainActivity::class.java)
            }else{
                initGoActivity(this@SplashActivity, LoginActivity::class.java)
            }
        }
    }
}