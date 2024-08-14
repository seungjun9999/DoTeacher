package com.example.doteacher.ui.dosunsang

import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.view.View
import androidx.annotation.RequiresApi
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.fragment.app.viewModels
import android.Manifest
import android.widget.Toast
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentDosunsangBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.dosunsang.viewmodel.DosunsangViewModel
import com.example.doteacher.ui.util.SingletonUtil
import com.google.zxing.integration.android.IntentIntegrator
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber
import java.time.LocalDate
import java.time.format.DateTimeFormatter

@AndroidEntryPoint
class DosunsangFragment : BaseFragment<FragmentDosunsangBinding>(R.layout.fragment_dosunsang) {

    private val dosunsangViewModel: DosunsangViewModel by viewModels()
    private val CAMERA_PERMISSION_REQUEST_CODE = 100

    override fun initView() {
        initData()
        clickEventListener()
        setTodayDate()
        observeViewModel()
    }

    override fun onResume() {
        super.onResume()
        SingletonUtil.user?.id?.let { dosunsangViewModel.getUserRobotState(it) }
        initData()
        setTodayDate()
    }

    private fun observeViewModel() {
        dosunsangViewModel.robotName.observe(viewLifecycleOwner) { name ->
            binding.robotD.text = name
        }

        dosunsangViewModel.connectionState.observe(viewLifecycleOwner) { state ->
            Timber.d("Connection state changed to: $state")
            updateUIBasedOnState(state)
        }
    }

    private fun updateUIBasedOnState(state: DosunsangViewModel.ConnectionState) {
        when (state) {
            DosunsangViewModel.ConnectionState.DISCONNECTED -> showDisconnectedState()
            DosunsangViewModel.ConnectionState.LOADING -> showLoadingState()
            DosunsangViewModel.ConnectionState.CONNECTED -> showConnectedState()
        }
    }


    private fun clickEventListener() {
        binding.noconnect.setOnClickListener {
            startRecommendation()
        }
        binding.photo.setOnClickListener {
            dosunsangViewModel.photo()
        }
        binding.gonext.setOnClickListener {
            dosunsangViewModel.gonext()
        }
        binding.qrcode.setOnClickListener {
            if (checkCameraPermission()) {
                startQRCodeScanner()
            } else {
                requestCameraPermission()
            }
        }
    }

    private fun checkCameraPermission(): Boolean {
        return ContextCompat.checkSelfPermission(
            requireContext(),
            Manifest.permission.CAMERA
        ) == PackageManager.PERMISSION_GRANTED
    }

    private fun requestCameraPermission() {
        ActivityCompat.requestPermissions(
            requireActivity(),
            arrayOf(Manifest.permission.CAMERA),
            CAMERA_PERMISSION_REQUEST_CODE
        )
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE) {
            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startQRCodeScanner()
            } else {
                Timber.d("카메라 권한 없음")
            }
        }
    }

    private fun startQRCodeScanner() {
        val integrator = IntentIntegrator.forSupportFragment(this)
        integrator.setDesiredBarcodeFormats(IntentIntegrator.QR_CODE)
        integrator.setPrompt("QR 코드를 스캔하세요")
        integrator.setCameraId(0)  // Use the rear camera
        integrator.setBeepEnabled(false)
        integrator.initiateScan()
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        val result = IntentIntegrator.parseActivityResult(requestCode, resultCode, data)
        if (result != null) {
            if (result.contents == null) {
                // QR 코드 스캔이 취소됨
            } else {
                // QR 코드 스캔 성공
                val scannedContent = result.contents

                // 스캔된 내용을 쉼표로 분리
                val parts = scannedContent.split(",")
                if (parts.size == 2) {
                    val robotName = parts[0].trim()
                    val number = parts[1].trim().toIntOrNull()

                    if (number != null) {
                        dosunsangViewModel.setRobotInfo(robotName, number)
                        Toast.makeText(context, "로봇 ID: $robotName, 번호: $number", Toast.LENGTH_LONG).show()
                    } else {
                        Toast.makeText(context, "잘못된 QR 코드 형식입니다.", Toast.LENGTH_LONG).show()
                    }
                } else {
                    Toast.makeText(context, "잘못된 QR 코드 형식입니다.", Toast.LENGTH_LONG).show()
                }
            }
        } else {
            super.onActivityResult(requestCode, resultCode, data)
        }
    }

    private fun startRecommendation() {
        dosunsangViewModel.robotNumber.value?.let { robotNumber ->
            SingletonUtil.user?.let { user ->
                dosunsangViewModel.recommend(robotNumber, user.userEmail)
            }
        } ?: run {
            Toast.makeText(context, "로봇 번호가 없습니다. QR 코드를 스캔해주세요.", Toast.LENGTH_SHORT).show()
        }
    }

    private fun showDisconnectedState() {
        binding.photo.isClickable = false
        binding.gonext.isClickable=false
        binding.connect.visibility = View.GONE
        binding.noconnect.visibility = View.VISIBLE
        binding.robotlottie.pauseAnimation()
        binding.robotlottie.visibility = View.GONE
        binding.sleeplottie.visibility = View.VISIBLE
        binding.sleeplottie.playAnimation()
        binding.loadinglottie.visibility = View.GONE
        binding.loadinglottie.pauseAnimation()
    }

    private fun showLoadingState() {
        binding.connect.visibility = View.GONE
        binding.noconnect.visibility = View.GONE
        binding.robotlottie.pauseAnimation()
        binding.robotlottie.visibility = View.GONE
        binding.sleeplottie.visibility = View.GONE
        binding.sleeplottie.pauseAnimation()
        binding.loadinglottie.visibility = View.VISIBLE
        binding.loadinglottie.playAnimation()
    }

    private fun showConnectedState() {
        binding.photo.isClickable = true
        binding.gonext.isClickable=true
        binding.connect.visibility = View.VISIBLE
        binding.noconnect.visibility = View.GONE
        binding.sleeplottie.pauseAnimation()
        binding.sleeplottie.visibility = View.GONE
        binding.loadinglottie.visibility = View.GONE
        binding.loadinglottie.pauseAnimation()
        binding.robotlottie.visibility = View.VISIBLE
        binding.robotlottie.playAnimation()
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