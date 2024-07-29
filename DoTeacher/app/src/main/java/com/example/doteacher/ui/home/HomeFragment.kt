package com.example.doteacher.ui.home

import androidx.activity.OnBackPressedCallback
import androidx.core.view.GravityCompat
import androidx.fragment.app.viewModels
import androidx.navigation.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentHomeBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.dialog.DialogFragment
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class HomeFragment : BaseFragment<FragmentHomeBinding>(R.layout.fragment_home) {
    private val homeViewModel: HomeViewModel by viewModels()
    private lateinit var homeAdapter: HomeAdapter

    override fun initView() {
        initData()
        setupOnBackPressed()
        setupRecyclerView()
        observeData()
        clickEventListener()
        showImageSliderDialog()
    }


    override fun onResume() {
        super.onResume()
        initData()
    }

    private fun initData() {
        binding.userData = SingletonUtil.user
    }

    private fun clickEventListener() {
        binding.menu.setOnClickListener {
            binding.drawerLayout.openDrawer(GravityCompat.START)
        }
        binding.btnViewall.setOnClickListener {
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_productFragment)
        }
        binding.btnGallery.setOnClickListener {
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_galleryFragment)
        }
        binding.btnChatbot.setOnClickListener {
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_chatgptFragment)
        }
        binding.btnDosunsang.setOnClickListener {
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_btn_dosunsangFragment)
        }

        binding.tvMenuPreference.setOnClickListener {
           view?.findNavController()?.navigate(R.id.action_mainFragment_to_preferenceActivity)
        }
    }

    private fun setupRecyclerView() {
        homeAdapter = HomeAdapter()
        binding.productRecycle.apply {
            layoutManager = LinearLayoutManager(context, LinearLayoutManager.HORIZONTAL, false)
            adapter = homeAdapter
        }
    }

    private fun setupOnBackPressed() {
        requireActivity().onBackPressedDispatcher.addCallback(
            this,
            object : OnBackPressedCallback(true) {
                override fun handleOnBackPressed() {
                    if (binding.drawerLayout.isDrawerOpen(GravityCompat.START)) {
                        binding.drawerLayout.closeDrawer(GravityCompat.START)
                    } else {
                        isEnabled = false
                        requireActivity().onBackPressed()
                    }
                }
            })
    }
    private fun showImageSliderDialog(){
        val dialog = DialogFragment()

        dialog.show(childFragmentManager,"DialogFragment")
    }


    private fun observeData() {
    }
}