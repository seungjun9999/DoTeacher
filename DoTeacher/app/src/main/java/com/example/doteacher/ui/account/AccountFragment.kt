package com.example.doteacher.ui.account

import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.constraintlayout.motion.widget.MotionLayout
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import androidx.navigation.fragment.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentAccountBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.login.LoginActivity
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.launch

@AndroidEntryPoint
class AccountFragment : BaseFragment<FragmentAccountBinding>(R.layout.fragment_account) {

    private val viewModel: AccountViewModel by viewModels()
    private var selectedImageUri: Uri? = null

    private val getContent = registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
        uri?.let {
            selectedImageUri = it
            binding.imgAccountUser.setImageURI(it)
            binding.imgAccountUser.visibility = View.VISIBLE
            uploadImageToS3(it)
        }
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initView()
        observeViewModel()
    }

    override fun initView() {
        binding.btnNext.setOnClickListener {
            val email = binding.etvUseremail.text.toString()
            val password = binding.etvUserpass.text.toString()
            if (email.isNotEmpty() && password.isNotEmpty()) {
                viewModel.setEmailAndPassword(email, password)
                (binding.root as MotionLayout).transitionToEnd()
            } else {
                Toast.makeText(context, "이메일과 비밀번호를 입력해주세요.", Toast.LENGTH_SHORT).show()
            }
        }

        binding.imgTologin.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }

        binding.btnPrevious.setOnClickListener {
            (binding.root as MotionLayout).transitionToStart()
        }

        (binding.root as MotionLayout).setTransitionListener(object : MotionLayout.TransitionListener {
            override fun onTransitionStarted(motionLayout: MotionLayout?, startId: Int, endId: Int) {}
            override fun onTransitionChange(motionLayout: MotionLayout?, startId: Int, endId: Int, progress: Float) {}
            override fun onTransitionCompleted(motionLayout: MotionLayout?, currentId: Int) {
                if (currentId == R.id.end) {
                    showProfileFieldsWithAnimation()
                } else if (currentId == R.id.start) {
                    resetToInitialState()
                }
            }
            override fun onTransitionTrigger(motionLayout: MotionLayout?, triggerId: Int, positive: Boolean, progress: Float) {}
        })

        binding.btnAccount.setOnClickListener {
            val nickname = binding.etvNickname.text.toString()
            if (nickname.isNotEmpty()) {
                viewModel.setNickname(nickname)
                viewModel.register()
            } else {
                Toast.makeText(context, "닉네임을 입력해주세요.", Toast.LENGTH_SHORT).show()
            }
        }
        binding.tvChooseProfile.setOnClickListener {
            getContent.launch("image/*")
        }
    }

    private fun showProfileFieldsWithAnimation() {
        binding.ctvProfileSetup.visibility = View.VISIBLE
        binding.imgAccountUser.visibility = View.VISIBLE
        binding.tvChooseProfile.visibility = View.VISIBLE
        binding.etvNickname.visibility = View.VISIBLE
        binding.btnAccount.visibility = View.VISIBLE
        binding.btnNext.visibility = View.GONE
        binding.btnPrevious.visibility = View.VISIBLE

        binding.etvUseremail.isEnabled = false
        binding.etvUserpass.isEnabled = false

        binding.etvNickname.requestFocus()
    }

    private fun resetToInitialState() {
        binding.ctvProfileSetup.visibility = View.GONE
        binding.imgAccountUser.visibility = View.GONE
        binding.tvChooseProfile.visibility = View.GONE
        binding.etvNickname.visibility = View.GONE
        binding.btnAccount.visibility = View.GONE
        binding.btnNext.visibility = View.VISIBLE
        binding.btnPrevious.visibility = View.GONE

        binding.etvUseremail.isEnabled = true
        binding.etvUserpass.isEnabled = true

        binding.etvUseremail.requestFocus()
    }


    private fun observeViewModel() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewModel.registrationState.collect { state ->
                when (state) {
                    is RegistrationState.Loading -> showLoading()
                    is RegistrationState.Success -> {
                        hideLoading()
                        Toast.makeText(context, "회원가입 성공!", Toast.LENGTH_SHORT).show()
                        val intent = Intent(requireContext(), LoginActivity::class.java).apply {
                            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
                        }
                        startActivity(intent)
                        requireActivity().finish()
                    }
                    is RegistrationState.Error -> {
                        hideLoading()
                        Toast.makeText(context, state.message, Toast.LENGTH_SHORT).show()
                    }
                    is RegistrationState.ImageUploaded -> {
                        hideLoading()
                        //Toast.makeText(context, "이미지 업로드 성공", Toast.LENGTH_SHORT).show()
                    }
                    else -> hideLoading()
                }
            }
        }
    }

    private fun uploadImageToS3(imageUri: Uri) {
        showLoading()
        viewModel.uploadImageToS3(requireContext(), imageUri)
    }

    private fun showLoading() {
        // 로딩 표시 구현
    }

    private fun hideLoading() {
        // 로딩 숨기기 구현
    }
}