package com.example.doteacher.ui.dosunsang

import android.view.View
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
    private var isRobotActive = false

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
            toggleRobotState()
        }
        binding.photo.setOnClickListener {
            dosunsangViewModel.photo()
        }
        binding.gonext.setOnClickListener {
            dosunsangViewModel.gonext()
        }
    }

    private fun toggleRobotState() {
        if (isRobotActive) {
            // 로봇이 활성화 상태일 때 슬립 모드로 전환
            binding.robotlottie.pauseAnimation()
            binding.robotlottie.visibility = View.GONE
            binding.sleeplottie.visibility = View.VISIBLE
            binding.sleeplottie.playAnimation()
        } else {
            // 로봇이 슬립 상태일 때 활성화 모드로 전환
            binding.sleeplottie.pauseAnimation()
            binding.sleeplottie.visibility = View.GONE
            binding.robotlottie.visibility = View.VISIBLE
            binding.robotlottie.playAnimation()

            // 여기에 recommend 함수 호출
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

        isRobotActive = !isRobotActive
    }

    private fun initData(){
        binding.userData = SingletonUtil.user
        binding.sleeplottie.visibility = View.VISIBLE
        binding.robotlottie.visibility = View.GONE
        binding.sleeplottie.playAnimation()
        binding.marquee.isSelected = true
        binding.marquee.requestFocus()
    }
}