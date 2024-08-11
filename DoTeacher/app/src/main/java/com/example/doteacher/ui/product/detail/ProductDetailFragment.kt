package com.example.doteacher.ui.product.detail

import android.os.Bundle
import androidx.navigation.findNavController
import com.example.doteacher.R
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.databinding.FragmentProductDetailBinding
import com.example.doteacher.ui.base.BaseFragment
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class ProductDetailFragment : BaseFragment<FragmentProductDetailBinding>(R.layout.fragment_product_detail) {

    private var productData : ProductData? =null

    override fun onCreate(savedInstanceState: Bundle?) {
         super.onCreate(savedInstanceState)
        arguments?.let {
            productData = it.getParcelable("productData")
        }
    }


    override fun initView(){
        binding.productData = productData
        binding.backProduct.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }
    }

    companion object{
        fun newInstance(productData: ProductData) = ProductDetailFragment().apply {
            arguments = Bundle().apply {
                putParcelable("productData",productData)
            }
        }
    }




}