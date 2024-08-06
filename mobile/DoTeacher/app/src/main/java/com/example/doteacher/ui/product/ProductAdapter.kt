package com.example.doteacher.ui.product

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.databinding.AllProductsItemsBinding
import com.example.doteacher.databinding.ProductListItemBinding
import com.example.doteacher.ui.util.DiffUtilCallback

class ProductAdapter :
    ListAdapter<ProductData, ProductAdapter.ViewHolder>(DiffUtilCallback<ProductData>()) {

   private var onItemClick:((ProductData)->Unit)? = null

    class ViewHolder(private val binding: AllProductsItemsBinding) :
        RecyclerView.ViewHolder(binding.root) {
        fun bind(productData: ProductData) {
            binding.productData = productData
            binding.executePendingBindings()
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val binding = AllProductsItemsBinding.inflate(LayoutInflater.from(parent.context),parent,false)
        return ViewHolder(binding)
    }

    override fun onBindViewHolder(holder:ViewHolder, position: Int) {
        holder.bind(getItem(position))
        holder.itemView.setOnClickListener {
            onItemClick?.let {
                it(getItem(position))
            }
        }
    }

    fun setOnItemClickListener(listener: (ProductData) ->Unit){
        onItemClick = listener
    }


}