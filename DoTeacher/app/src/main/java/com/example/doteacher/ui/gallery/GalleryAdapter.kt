package com.example.doteacher.ui.gallery

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.example.doteacher.data.model.PhotoData
import com.example.doteacher.databinding.MyphotoItemBinding
import com.example.doteacher.ui.util.DiffUtilCallback

class GalleryAdapter() :
    ListAdapter<PhotoData, GalleryAdapter.ViewHolder>(
        DiffUtilCallback<PhotoData>()
    ) {

    private var onItemClick: ((PhotoData) -> Unit)? = null


    class ViewHolder(private val binding: MyphotoItemBinding) : RecyclerView.ViewHolder(binding.root) {
        fun bind(photoData: PhotoData) {
            binding.photoData = photoData
            binding.executePendingBindings()
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val binding = MyphotoItemBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return ViewHolder(binding)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        holder.bind(getItem(position))
        holder.itemView.setOnClickListener {
            onItemClick?.let {
                it(getItem(position))
            }
        }
    }

    fun setOnItemClickListener(listener:(PhotoData)->Unit){
        onItemClick = listener
    }
}