package com.example.doteacher.ui.gallery

import android.graphics.Color
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.fragment.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentGalleryBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.util.SingletonUtil


class GalleryFragment : BaseFragment<FragmentGalleryBinding>(R.layout.fragment_gallery) {

    private var isPhotoListActive = true

    override fun initView() {
        intitData()
        clickEventListener()
    }

    private fun intitData(){
        binding.userData = SingletonUtil.user
    }

    private fun clickEventListener(){
        binding.btnReturnHome.setOnClickListener {
            this.findNavController().popBackStack()
        }
        binding.bottomSheetLayout.btnPhotoList.setOnClickListener {
            togglePhotoListBtn()
        }
    }

    private fun togglePhotoListBtn(){
        isPhotoListActive = !isPhotoListActive

        if (isPhotoListActive) {
            binding.bottomSheetLayout.btnPhotoList.setBackgroundResource(R.drawable.gallery_orange_btn)
            binding.bottomSheetLayout.btnPhotoList.setTextColor(Color.parseColor("#f7b84b"))
        } else {
            binding.bottomSheetLayout.btnPhotoList.background = null
            binding.bottomSheetLayout.btnPhotoList.setTextColor(Color.parseColor("#88878a"))
        }
    }
}