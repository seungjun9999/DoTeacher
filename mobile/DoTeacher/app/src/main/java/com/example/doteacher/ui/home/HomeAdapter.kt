package com.example.doteacher.ui.home

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.doteacher.R
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.databinding.ProductListItemBinding

class HomeAdapter : RecyclerView.Adapter<HomeAdapter.ProductViewHolder>() {

    private val products = List(10) { ProductData("황소", R.drawable.hwangso,"황소임  ㅋㅋ ㄹㅇ ㅋㅋ") }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ProductViewHolder {
        val binding = ProductListItemBinding.inflate(LayoutInflater.from(parent.context), parent, false)
        return ProductViewHolder(binding)
    }

    override fun onBindViewHolder(holder: ProductViewHolder, position: Int) {
        holder.bind(products[position])
    }

    override fun getItemCount(): Int = products.size

    class ProductViewHolder(private val binding: ProductListItemBinding) : RecyclerView.ViewHolder(binding.root) {
        fun bind(product: ProductData) {
            binding.productData = product
            binding.executePendingBindings()
        }
    }
}