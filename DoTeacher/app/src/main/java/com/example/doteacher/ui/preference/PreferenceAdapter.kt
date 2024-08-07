package com.example.doteacher.ui.preference

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.doteacher.databinding.PreferItemImageBinding

class PreferenceAdapter(
    private val preferences: List<Pair<Int, String>>,
    private val onItemClick: (Int, String) -> Unit
) : RecyclerView.Adapter<PreferenceAdapter.ViewHolder>() {

    private val selectedItems = mutableSetOf<String>()

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val binding = PreferItemImageBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return ViewHolder(binding)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        holder.bind(preferences[position])
    }

    override fun getItemCount(): Int = preferences.size

    fun updateSelectedItems(selectedPreferences: Set<String>) {
        selectedItems.clear()
        selectedItems.addAll(selectedPreferences)
        notifyDataSetChanged()
    }

    inner class ViewHolder(private val binding: PreferItemImageBinding) : RecyclerView.ViewHolder(binding.root) {
        fun bind(preference: Pair<Int, String>) {
            binding.imgPreferItemImage.setImageResource(preference.first)
            binding.selectedImageBordersmall.visibility = if (selectedItems.contains(preference.second)) View.VISIBLE else View.INVISIBLE
            binding.root.setOnClickListener {
                onItemClick(preference.first, preference.second)
            }
        }
    }
}