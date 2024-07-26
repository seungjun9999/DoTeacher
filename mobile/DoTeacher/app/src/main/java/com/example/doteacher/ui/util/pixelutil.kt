package com.example.doteacher.ui.util

import android.content.res.Resources
import android.util.TypedValue

private val displayMetrics
    get() = Resources.getSystem().displayMetrics

fun dpToPx(dp: Int) =
    TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, dp.toFloat(), displayMetrics).toInt()


val Number.dpToPx: Int
    get() = dpToPx(this.toInt())