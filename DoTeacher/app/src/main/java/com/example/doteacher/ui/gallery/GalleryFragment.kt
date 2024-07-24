package com.example.doteacher.ui.gallery

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
    }
}