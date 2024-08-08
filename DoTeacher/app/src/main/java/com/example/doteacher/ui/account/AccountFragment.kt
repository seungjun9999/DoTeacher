package com.example.doteacher.ui.account

import android.net.Uri
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.fragment.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentAccountBinding
import com.example.doteacher.ui.base.BaseFragment
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.launch

@AndroidEntryPoint
class AccountFragment : BaseFragment<FragmentAccountBinding>(R.layout.fragment_account) {

    private val viewModel: AccountViewModel by viewModels()

    private val getContent = registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
        uri?.let {
            binding.imgAccountUser.setImageURI(it)
            viewModel.setImageUrl(it.toString())
        }
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initView()
        observeViewModel()
    }

    override fun initView() {
        binding.button.setOnClickListener {
            val email = binding.etvUseremail.text.toString()
            val password = binding.etvUserpass.text.toString()
            if (email.isNotEmpty() && password.isNotEmpty()) {
                viewModel.setEmailAndPassword(email, password)
                showProfileFields()
            } else {
                Toast.makeText(context, "이메일과 비밀번호를 입력해주세요.", Toast.LENGTH_SHORT).show()
            }
        }

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

    private fun showProfileFields() {
        binding.imgAccountUser.visibility = View.VISIBLE
        binding.tvChooseProfile.visibility = View.VISIBLE
        binding.etvNickname.visibility = View.VISIBLE
        binding.btnAccount.visibility = View.VISIBLE
    }

    private fun observeViewModel() {
        viewLifecycleOwner.lifecycleScope.launch {
            viewModel.registrationState.collect { state ->
                when (state) {
                    is RegistrationState.Loading -> showLoading()
                    is RegistrationState.Success -> {
                        hideLoading()
                        Toast.makeText(context, "회원가입 성공!", Toast.LENGTH_SHORT).show()
                        findNavController().navigate(R.id.action_accountActivity_to_loginActivity)
                    }
                    is RegistrationState.Error -> {
                        hideLoading()
                        Toast.makeText(context, "오류: ${state.message}", Toast.LENGTH_SHORT).show()
                    }
                    else -> hideLoading()
                }
            }
        }
    }

    private fun showLoading() {
        // 로딩 표시 구현
    }

    private fun hideLoading() {
        // 로딩 숨기기 구현
    }
}