package com.example.doteacher.ui.home

import androidx.fragment.app.viewModels
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentHomeBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint


@AndroidEntryPoint
class HomeFragment : BaseFragment<FragmentHomeBinding>(R.layout.fragment_home) {
    private val homeViewModel : HomeViewModel by viewModels()
    private lateinit var homeAdapter: HomeAdapter

    private val viewModel: HomeViewModel by viewModels()

    override fun initView() {
        initData()
    }

    override fun onResume() {
        super.onResume()
        initData()
    }


    private fun initData(){
        binding.userData = SingletonUtil.user
    }
}