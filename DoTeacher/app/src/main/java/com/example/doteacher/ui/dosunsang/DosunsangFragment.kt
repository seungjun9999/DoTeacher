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
import java.time.LocalDate
import java.time.format.DateTimeFormatter

@AndroidEntryPoint
class DosunsangFragment : BaseFragment<FragmentDosunsangBinding>(R.layout.fragment_dosunsang) {

    private val dosunsangViewModel: DosunsangViewModel by viewModels()
    private var isRobotActive = false

    override fun initView() {
        initData()
        clickEventListener()
        setTodayDate()
    }

    override fun onResume() {
        super.onResume()
        initData()
        setTodayDate()
    }

    private fun clickEventListener() {
        binding.noconnect.setOnClickListener {
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
            binding.connect.visibility = View.GONE
            binding.noconnect.visibility = View.VISIBLE
            binding.robotlottie.pauseAnimation()
            binding.robotlottie.visibility = View.GONE
            binding.sleeplottie.visibility = View.VISIBLE
            binding.sleeplottie.playAnimation()
            binding.connect.isClickable = false
        } else {
            binding.connect.visibility = View.VISIBLE
            binding.noconnect.visibility = View.GONE
            binding.sleeplottie.pauseAnimation()
            binding.sleeplottie.visibility = View.GONE
            binding.robotlottie.visibility = View.VISIBLE
            binding.robotlottie.playAnimation()

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

    private fun initData() {
        binding.userData = SingletonUtil.user
        binding.sleeplottie.visibility = View.VISIBLE
        binding.robotlottie.visibility = View.GONE
        binding.sleeplottie.playAnimation()
        binding.marquee.isSelected = true
        binding.marquee.requestFocus()
    }

    private fun setTodayDate() {
        val currentDate = LocalDate.now()
        val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd")
        val formattedDate = currentDate.format(formatter)
        binding.today.text = formattedDate
    }
}