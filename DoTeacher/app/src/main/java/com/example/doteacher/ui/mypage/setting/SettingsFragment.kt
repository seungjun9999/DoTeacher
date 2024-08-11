package com.example.doteacher.ui.settings

import android.app.AlertDialog
import android.content.Intent
import android.view.View
import android.widget.Toast
import androidx.fragment.app.viewModels
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentSettingBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.login.LoginActivity
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.mypage.setting.SettingsViewModel
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class SettingsFragment : BaseFragment<FragmentSettingBinding>(R.layout.fragment_setting) {


    private val settingsviewModel: SettingsViewModel by viewModels()


    override fun initView() {
        setupSettingsItems()
        observeWithdrawState()
    }

    override fun onResume() {
        super.onResume()
        hideBottomNavigation()
    }

    override fun onPause() {
        super.onPause()
        showBottomNavigation()
    }

    private fun hideBottomNavigation() {
        (activity as? MainActivity)?.hideBottomNavigation()
    }

    private fun showBottomNavigation() {
        (activity as? MainActivity)?.showBottomNavigation()
    }

    private fun setupSettingsItems() {
        with(binding) {
            itemLogout.apply {
                imgMypage.setImageResource(R.drawable.set_logout)
                tvMypageText.text="로그아웃"
                root.setOnClickListener {
                    Timber.d("Logout button clicked")
                    logout()
                }
            }
            itemSignout.apply {
                imgMypage.setImageResource(R.drawable.set_bye)
                tvMypageText.text="회원탈퇴"
                root.setOnClickListener {
                    Timber.d("탈퇴 button clicked")
                    showWithdrawConfirmationDialog()
                }
            }
        }
    }

    private fun showWithdrawConfirmationDialog() {
        AlertDialog.Builder(requireContext())
            .setTitle("회원 탈퇴")
            .setMessage("정말로 탈퇴하시겠습니까? 이 작업은 되돌릴 수 없습니다.")
            .setPositiveButton("탈퇴") { _, _ ->
                settingsviewModel.withdrawUser()
            }
            .setNegativeButton("취소", null)
            .show()
    }

    private fun observeWithdrawState() {
        settingsviewModel.withdrawState.observe(viewLifecycleOwner) { state ->
            when (state) {
                is SettingsViewModel.WithdrawState.Loading -> showLoading()
                is SettingsViewModel.WithdrawState.Success -> {
                    hideLoading()
                    Toast.makeText(context, "회원 탈퇴가 완료되었습니다.", Toast.LENGTH_SHORT).show()
                    navigateToLogin()
                }
                is SettingsViewModel.WithdrawState.Error -> {
                    hideLoading()
                    Toast.makeText(context, "회원 탈퇴 실패: ${state.message}", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }


    private fun showLoading() {
        binding.logoutlottie.visibility = View.VISIBLE
        binding.logoutlottie.playAnimation()
    }

    private fun hideLoading() {
        binding.logoutlottie.visibility = View.GONE
        binding.logoutlottie.cancelAnimation()
    }

    private fun navigateToLogin() {
        val intent = Intent(requireContext(), LoginActivity::class.java)
        intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
        startActivity(intent)
        activity?.finish()
    }

    private fun logout() {
        Timber.d("Logout function called")
        settingsviewModel.logout()
        SingletonUtil.user = null
        val intent = Intent(requireContext(), LoginActivity::class.java)
        intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
        startActivity(intent)
        activity?.finish()
    }
}