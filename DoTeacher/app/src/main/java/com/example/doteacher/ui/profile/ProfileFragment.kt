package com.example.doteacher.ui.profile

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.findNavController
import com.bumptech.glide.Glide.init
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentProfileBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.util.SingletonUtil


class ProfileFragment : BaseFragment<FragmentProfileBinding>(R.layout.fragment_profile) {
    override fun initView() {
        initData()
        clickEvent()
    }

    private fun initData(){
        binding.userData = SingletonUtil.user
    }

    private fun clickEvent(){
        binding.btnProfileToHome.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }
    }


}