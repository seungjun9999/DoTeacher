package com.example.doteacher.ui.util

import android.widget.ImageView
import android.widget.TextView
import androidx.databinding.BindingAdapter
import com.bumptech.glide.Glide
import com.bumptech.glide.load.resource.bitmap.CircleCrop
import com.example.doteacher.R
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@BindingAdapter("setSimpleTime")
fun setHour(textView: TextView, date: Long) {
    val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault())
    val toDate = sdf.format(Date(date))
    textView.text = toDate
}

@BindingAdapter("imageUrl")
fun loadImage(view: ImageView, url: String?) {
    if (!url.isNullOrEmpty()) {
        Glide.with(view.context)
            .load(url)
            .centerCrop()
            .into(view)
    }
}

@BindingAdapter("setCircleProfileImage")
fun setCircleProfileImage(imageView: ImageView, src: String?) {
    if(src.isNullOrEmpty()){
        imageView.setImageResource(R.drawable.mimitaya)
    }else{
        src.let {
            Glide.with(imageView).load(src)
                .override(100.dpToPx,100.dpToPx)
                .transform(CircleCrop())
                .into(imageView)
        }
    }

}
@BindingAdapter("imageResId")
fun bindImage(imageView: ImageView, resId: Int) {
    imageView.setImageResource(resId)
}
