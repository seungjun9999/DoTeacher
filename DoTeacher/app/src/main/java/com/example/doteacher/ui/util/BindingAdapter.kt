package com.example.doteacher.ui.util

import android.widget.ImageView
import android.widget.TextView
import androidx.databinding.BindingAdapter
import com.bumptech.glide.Glide
import com.bumptech.glide.load.resource.bitmap.CircleCrop
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@BindingAdapter("setSimpleTime")
fun setHour(textView: TextView, date: Long) {
    val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault())
    val toDate = sdf.format(Date(date))
    textView.text = toDate
}

@BindingAdapter("setCircleProfileImage")
fun setCircleProfileImage(imageView: ImageView, src: String?) {
    src?.let {
        Glide.with(imageView).load(src)
            .override(100.dpToPx,100.dpToPx)
            .transform(CircleCrop())
            .into(imageView)
    }
}
