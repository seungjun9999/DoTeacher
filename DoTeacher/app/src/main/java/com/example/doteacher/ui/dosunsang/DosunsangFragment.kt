package com.example.doteacher.ui.dosunsang

import android.os.Build
import android.view.View
import androidx.annotation.RequiresApi
import androidx.fragment.app.viewModels
import com.example.doteacher.R
import com.example.doteacher.data.model.param.UserParam
import com.example.doteacher.databinding.FragmentDosunsangBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.dosunsang.viewmodel.ConnectionState
import com.example.doteacher.ui.dosunsang.viewmodel.DosunsangViewModel
import com.example.doteacher.ui.util.SingletonUtil
import com.example.doteacher.ui.util.SingletonUtil.user
import dagger.hilt.android.AndroidEntryPoint
import java.time.LocalDate
import java.time.format.DateTimeFormatter

@AndroidEntryPoint
class DosunsangFragment : BaseFragment<FragmentDosunsangBinding>(R.layout.fragment_dosunsang) {

    private val dosunsangViewModel: DosunsangViewModel by viewModels()
    override fun initView() {
        initData()
        clickEventListener()
        setTodayDate()
        observeViewModel()
        dosunsangViewModel.initConnection(requireContext())
    }

    override fun onResume() {
        super.onResume()
        initData()
        setTodayDate()
    }

    private fun observeViewModel() {
        dosunsangViewModel.connectionState.observe(viewLifecycleOwner) { state ->
            when (state) {
                ConnectionState.DISCONNECTED -> {
                    binding.noconnect.visibility = View.VISIBLE
                    binding.connect.visibility = View.GONE
                    binding.sleeplottie.visibility = View.VISIBLE
                    binding.robotlottie.visibility = View.GONE
                    binding.loadinglottie.visibility = View.GONE
                }
                ConnectionState.CONNECTING -> {
                    binding.noconnect.visibility = View.GONE
                    binding.connect.visibility = View.GONE
                    binding.sleeplottie.visibility = View.GONE
                    binding.robotlottie.visibility = View.GONE
                    binding.loadinglottie.visibility = View.VISIBLE
                }
                ConnectionState.CONNECTED -> {
                    binding.noconnect.visibility = View.GONE
                    binding.connect.visibility = View.VISIBLE
                    binding.sleeplottie.visibility = View.GONE
                    binding.robotlottie.visibility = View.VISIBLE
                    binding.loadinglottie.visibility = View.GONE
                    dosunsangViewModel.saveConnectionState(requireContext(), true)
                }
            }
        }

        dosunsangViewModel.serverMessage.observe(viewLifecycleOwner) { message ->
            when (message) {
                "ack start" -> {
                    binding.loadinglottie.visibility = View.GONE
                    binding.loadinglottie.cancelAnimation()
                }
                "ack end" -> {
                    dosunsangViewModel.saveConnectionState(requireContext(), false)
                }
            }
        }
    }

    private fun clickEventListener() {
        binding.noconnect.setOnClickListener {
            binding.loadinglottie.visibility = View.VISIBLE
            binding.loadinglottie.playAnimation()
            dosunsangViewModel.toggleConnection(requireContext())
        }
        binding.photo.setOnClickListener {
            dosunsangViewModel.photo()
        }
        binding.gonext.setOnClickListener {
            dosunsangViewModel.gonext()
        }
    }


    private fun initData() {
        binding.userData = SingletonUtil.user
        binding.marquee.isSelected = true
        binding.marquee.requestFocus()
    }

    @RequiresApi(Build.VERSION_CODES.O)
    private fun setTodayDate() {
        val currentDate = LocalDate.now()
        val formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd")
        val formattedDate = currentDate.format(formatter)
        binding.today.text = formattedDate
    }
}