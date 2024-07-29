package com.example.doteacher.ui.chatgpt

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import androidx.recyclerview.widget.RecyclerView.ViewHolder
import com.example.doteacher.databinding.ItemLeftChatBinding
import com.example.doteacher.databinding.ItemRightChatBinding
import com.example.doteacher.ui.product.ProductAdapter
import com.example.doteacher.ui.util.DiffUtilCallback

class ChatGptAdapter:ListAdapter<GptMessageItem, ViewHolder>(
    DiffUtilCallback<GptMessageItem>()
) {

    override fun getItemViewType(position: Int): Int {
        return if(getItem(position).role =="user"){
            USER
        }else{
            GPT
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        if (viewType == USER) {
            val binding = ItemRightChatBinding.inflate(LayoutInflater.from(parent.context), parent, false)
            return UserViewHolder(binding)
        } else {
            val binding = ItemLeftChatBinding.inflate(LayoutInflater.from(parent.context), parent, false)
            return GptViewHolder(binding)
        }
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        when (holder) {
            is UserViewHolder -> {
                holder.bind(getItem(position))
            }

            is GptViewHolder -> {
                holder.bind(getItem(position))
            }
        }
    }

    class UserViewHolder(
        val binding: ItemRightChatBinding
    ) : RecyclerView.ViewHolder(binding.root) {
        fun bind(message: GptMessageItem) {
            binding.textChat.text = message.content
        }
    }

    class GptViewHolder(
        val binding: ItemLeftChatBinding
    ) : RecyclerView.ViewHolder(binding.root) {
        fun bind(message: GptMessageItem) {
            binding.textChat.text = message.content
        }
    }

    companion object {
        const val USER = 0
        const val GPT = 1
    }
}