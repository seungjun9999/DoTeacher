package com.example.doteacher.ui.product.detail

import android.os.Build.VERSION_CODES.P
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.fragment.navArgs
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
    }

    companion object{
        fun newInstance(productData: ProductData) = ProductDetailFragment().apply {
            arguments = Bundle().apply {
                putParcelable("productData",productData)
            }
        }
    }




}