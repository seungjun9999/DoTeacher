package com.example.doteacher.ui.mypage

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.fragment.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentMyPageBinding
import com.example.doteacher.ui.base.BaseFragment
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {

    override fun initView() {
        setupMyPageItems()
    }

    private fun setupMyPageItems() {
        with(binding) {
            itemProfile.apply {
                imgMypage.setImageResource(R.drawable.real_profile) // 프로필 아이콘 리소스
                tvMypageText.text = "프로필"
                root.setOnClickListener {
                    findNavController().navigate(R.id.action_mypageFragment_to_profileFragment)
                }
            }
            itemPreferences.apply {
                imgMypage.setImageResource(R.drawable.edit_icon) // 취향수정 아이콘 리소스
                tvMypageText.text = "취향수정"
                root.setOnClickListener {
                    findNavController().navigate(R.id.action_mypageFragment_to_preferenceActivity)
                }
            }
            itemGuide.apply {
                imgMypage.setImageResource(R.drawable.tuto) // 사용법 안내 아이콘 리소스
                tvMypageText.text = "사용법 안내"
            }
            itemSettings.apply {
                imgMypage.setImageResource(R.drawable.setting) // 설정 아이콘 리소스
                tvMypageText.text = "설정"
            }
        }
    }
}