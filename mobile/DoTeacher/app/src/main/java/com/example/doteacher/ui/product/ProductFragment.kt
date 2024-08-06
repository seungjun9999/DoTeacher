package com.example.doteacher.ui.product

import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.findNavController
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.GridLayoutManager
import androidx.recyclerview.widget.StaggeredGridLayoutManager
import com.example.doteacher.R
import com.example.doteacher.data.model.ProductData
import com.example.doteacher.databinding.FragmentProductBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber


@AndroidEntryPoint
class ProductFragment : BaseFragment<FragmentProductBinding>(R.layout.fragment_product) {

    private val productViewModel : ProductViewModel by viewModels()
    private lateinit var productAdapter: ProductAdapter


    override fun initView() {
        initData()
        setupRecyclerView()
        initAdapter()
        clickEventListener()
        observeProductData()
        productViewModel.getAllProducts()
    }

    private fun initData(){
        binding.userData = SingletonUtil.user
    }

    private fun setupRecyclerView(){
        binding.allProductRecycle.apply {
            layoutManager = StaggeredGridLayoutManager(2,StaggeredGridLayoutManager.VERTICAL)
        }
    }

    private fun initAdapter() {
        productAdapter = ProductAdapter().apply {
            setOnItemClickListener { productData ->
                Timber.d("Product clicked: $productData")
                findNavController().navigate(
                    R.id.action_productFragment_to_productdetailFragment,
                    bundleOf("productData" to productData)
                )
            }
        }
        binding.allProductRecycle.adapter = productAdapter
    }

    private fun clickEventListener() {
        binding.btnReturnHome.setOnClickListener {
            it.findNavController().popBackStack()
        }
    }

    private fun observeProductData() {
        productViewModel.allProducts.observe(viewLifecycleOwner){  allproduct ->
            productAdapter.submitList(allproduct)
        }
    }



    override fun onResume() {
        super.onResume()
        initData()
        //productViewModel.getAllProducts()
    }

}