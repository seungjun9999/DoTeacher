package com.example.doteacher.ui.util

import android.app.Activity
import android.content.Intent

fun <T : Activity>initGoActivity(activity : Activity, otherActivity : Class<T>){
    activity.apply {
        startActivity(Intent(this, otherActivity))
        finish()
    }
}