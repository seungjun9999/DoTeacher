package com.example.doteacher.ui.product.detail

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.fragment.navArgs
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentProductDetailBinding
import com.example.doteacher.ui.base.BaseFragment


class ProductDetailFragment : BaseFragment<FragmentProductDetailBinding>(R.layout.fragment_product_detail) {


    override fun initView(){
        initData()
    }

    private fun initData(){
        val safeArgs : ProductDetailFragmentArgs by navArgs()
        binding.productData = safeArgs.productData
    }



}