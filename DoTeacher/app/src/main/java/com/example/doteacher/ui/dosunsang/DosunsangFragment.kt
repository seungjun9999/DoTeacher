package com.example.doteacher.ui.dosunsang

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
    }

    private fun initData(){

    }
}

