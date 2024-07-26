package com.example.doteacher.ui

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentDosunsangBinding
import com.example.doteacher.ui.base.BaseFragment
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class DosunsangFragment : BaseFragment<FragmentDosunsangBinding>(R.layout.fragment_dosunsang){

    override fun initView(){
        initData()
        clickEventListener()
    }

    override fun onResume() {
        super.onResume()
        initData()
    }

    private fun clickEventListener(){
        binding.dosunsangToHome.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }
    }

    private fun initData(){

    }
}

