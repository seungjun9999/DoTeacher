package com.example.doteacher.ui.dosunsang

import androidx.fragment.app.viewModels
import com.example.doteacher.R
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.databinding.FragmentDosunsangBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.dosunsang.viewmodel.DosunsangViewModel
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.SingletonUtil.user
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class DosunsangFragment : BaseFragment<FragmentDosunsangBinding>(R.layout.fragment_dosunsang){

    private val dosunsangViewModel: DosunsangViewModel by viewModels()

    override fun initView(){
        initData()
        clickEventListener()
    }

    override fun onResume() {
        super.onResume()
        initData()

    }

    private fun clickEventListener(){
        binding.connect.setOnClickListener {
            val userParam = UserParam(
                userEmail = SingletonUtil.user!!.userEmail,
                userName = SingletonUtil.user!!.userName,
                userImage = SingletonUtil.user!!.userImage,
                preferences = SingletonUtil.user!!.preferences,
                password = "pass",
                userTuto = true,
                prefSelect = false
            )

            dosunsangViewModel.recommend(userParam)
        }
    }

    private fun initData(){
        binding.userData = SingletonUtil.user
        binding.robotlottie.playAnimation()
    }
}

