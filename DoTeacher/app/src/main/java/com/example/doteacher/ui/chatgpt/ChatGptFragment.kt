package com.example.doteacher.ui.chatgpt

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentChatGptBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.home.HomeFragment
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class ChatGptFragment : BaseFragment<FragmentChatGptBinding>(R.layout.fragment_chat_gpt) {

    override fun initView() {
        initData()
        clickEventListener()
    }

    override fun onResume() {
        super.onResume()
        initData()
    }

    private fun initData() {

    }

    private fun clickEventListener(){
        binding.chatToHome.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }
    }
}