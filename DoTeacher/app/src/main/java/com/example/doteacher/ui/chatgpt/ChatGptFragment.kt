package com.example.doteacher.ui.chatgpt

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentChatGptBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.chatgpt.viewmodel.ChatGptViewModel
import com.example.doteacher.ui.home.HomeFragment
import com.example.doteacher.ui.util.hideKeyboard
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch


@AndroidEntryPoint
class ChatGptFragment : BaseFragment<FragmentChatGptBinding>(R.layout.fragment_chat_gpt) {

    private val gptViewModel: ChatGptViewModel by viewModels()

    private lateinit var gptAdapter: ChatGptAdapter

    override fun initView() {
        initData()
        clickEventListener()
        initAdapter()
        sendMessage()
        observeMessage()
        observeGptDataSuccess()
    }

    override fun onResume() {
        super.onResume()
        initData()
    }

    private fun initAdapter(){
        gptAdapter = ChatGptAdapter()
        binding.rvChatgpt.adapter = gptAdapter
    }

    private fun initData() {

    }

    private fun observeMessage(){
        gptViewModel.gptData.observe(viewLifecycleOwner){
            gptAdapter.submitList(it)
        }
    }

    private fun observeGptDataSuccess(){
        gptViewModel.gptDataSuccess.observe(viewLifecycleOwner){
            lifecycleScope.launch {
                delay(300L)
                binding.rvChatgpt.scrollToPosition(0)
            }
        }
    }

    private fun sendMessage(){
        binding.imgSend.setOnClickListener {
            val value = binding.etChat.text.toString()
            gptViewModel.gptData.addAll(listOf(GptMessageItem("user",value)))
            requireActivity().hideKeyboard(binding.etChat)
            gptViewModel.sendChat(value)
            binding.etChat.clearFocus()
            binding.etChat.text.clear()
        }
    }

    private fun clickEventListener(){
        binding.chatToHome.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }
    }
}