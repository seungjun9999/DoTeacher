package com.example.doteacher.ui.settings

import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentSettingBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.main.MainActivity
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class SettingsFragment : BaseFragment<FragmentSettingBinding>(R.layout.fragment_setting) {

    override fun initView() {
        setupSettingsItems()
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
                root.setOnClickListener {

                }
            }
            itemWithdraw.apply {
                root.setOnClickListener {

                }
            }
        }
    }
}