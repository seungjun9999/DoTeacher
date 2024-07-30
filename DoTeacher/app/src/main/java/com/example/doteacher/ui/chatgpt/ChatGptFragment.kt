package com.example.doteacher.ui.chatgpt

import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentChatGptBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.chatgpt.viewmodel.ChatGptViewModel
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
        val layoutManager = LinearLayoutManager(context)
        layoutManager.stackFromEnd = true
        binding.rvChatgpt.layoutManager = layoutManager
    }

    private fun initData() {
        // 필요한 초기 데이터 설정
    }

    private fun observeMessage(){
        gptViewModel.gptData.observe(viewLifecycleOwner) { messages ->
            gptAdapter.submitList(messages.toList()) {
                scrollToBottom()
            }
        }
    }

    private fun observeGptDataSuccess(){
        gptViewModel.gptDataSuccess.observe(viewLifecycleOwner) {
            lifecycleScope.launch {
                delay(300L)
                scrollToBottom()
            }
        }
    }

    private fun sendMessage(){
        binding.imgSend.setOnClickListener {
            val value = binding.etChat.text.toString().trim()
            if (value.isNotEmpty()) {
                gptViewModel.addUserMessage(value)
                requireActivity().hideKeyboard(binding.etChat)
                gptViewModel.sendChat(value)
                binding.etChat.clearFocus()
                binding.etChat.text.clear()
                scrollToBottom()
            }
        }
    }

    private fun scrollToBottom() {
        binding.rvChatgpt.post {
            val itemCount = gptAdapter.itemCount
            if (itemCount > 0) {
                binding.rvChatgpt.smoothScrollToPosition(itemCount - 1)
            }
        }
    }

    private fun clickEventListener(){
        binding.chatToHome.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }
    }
}