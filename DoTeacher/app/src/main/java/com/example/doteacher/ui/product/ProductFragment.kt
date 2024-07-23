package com.example.doteacher.ui.product

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.GridLayoutManager
import com.example.doteacher.R
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.databinding.FragmentProductBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.home.HomeAdapter
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber


@AndroidEntryPoint
class ProductFragment : BaseFragment<FragmentProductBinding>(R.layout.fragment_product) {

    private val productViewModel : ProductViewModel by viewModels()
    private lateinit var productAdapter: ProductAdapter


    override fun initView() {
        initData()
        initAdapter()
        setupRecyclerView()
        loadProducts()
    }


    private fun initData(){
        binding.userData = SingletonUtil.user
    }

    private fun setupRecyclerView(){
        productAdapter = ProductAdapter()
        binding.allProductRecycle.apply {
            adapter = productAdapter
            layoutManager = GridLayoutManager(context,3)
        }
    }
    private fun loadProducts() {
        val products = List(10) {
            ProductData("황소", R.drawable.hwangso,"황소임 ㅋㅋ")
        }
        productAdapter.submitList(products)
    }

    private fun initAdapter() {
        productAdapter = ProductAdapter()
        binding.allProductRecycle.adapter = productAdapter

        productAdapter.setOnItemClickListener { productData ->
            Timber.d("Product clicked: $productData")
            this@ProductFragment.findNavController().navigate(
                R.id.action_productFragment_to_productdetailFragment,
                bundleOf("productData" to productData)
            )
        }
    }



    override fun onResume() {
        super.onResume()
        initData()
    }

}