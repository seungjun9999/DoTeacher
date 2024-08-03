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
import com.example.doteacher.ui.home.viewmodel.HomeViewModel
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class HomeFragment : BaseFragment<FragmentHomeBinding>(R.layout.fragment_home), DialogFragment.DialogListener {
    private val homeViewModel: HomeViewModel by viewModels()
    private lateinit var homeAdapter: HomeAdapter

    override fun initView() {
        initData()
        setupOnBackPressed()
        observeData()
        clickEventListener()
        checkAndShowDialog()
        initAdapter()
        observeUserTutoUpdate()
    }

    override fun onResume() {
        super.onResume()
        initData()
        homeViewModel.getRandomProducts()
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
        binding.tvMenuProfile.setOnClickListener {
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_profileFragment)
        }
        binding.imgProfile.setOnClickListener{
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_profileFragment)
        }
    }

    private fun initAdapter() {
        homeAdapter = HomeAdapter()
        binding.productRecycle.apply {
            layoutManager = LinearLayoutManager(context, LinearLayoutManager.HORIZONTAL, false)
            adapter = homeAdapter
        }
        homeViewModel.getRandomProducts()
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

    private fun checkAndShowDialog() {
        if (SingletonUtil.user?.userTuto == false) {
            showImageSliderDialog()
        }
    }

    private fun showImageSliderDialog() {
        val dialog = DialogFragment()
        dialog.setDialogListener(this)
        dialog.show(childFragmentManager, "DialogFragment")
    }

    override fun onDialogClosed(neverShowAgain: Boolean) {
        Timber.d("Dialog closed with neverShowAgain: $neverShowAgain")
        if (neverShowAgain) {
            homeViewModel.updateUserTuto()
        }
    }

    private fun observeData() {
        homeViewModel.randomProducts.observe(viewLifecycleOwner) { products ->
            homeAdapter.submitList(products)
        }
    }

    private fun observeUserTutoUpdate() {
        homeViewModel.userTutoUpdated.observe(viewLifecycleOwner) { updated ->
            if (updated) {
                Timber.d("UserTuto updated successfully")
                // UI 업데이트 또는 다른 필요한 작업 수행
            } else {
                Timber.d("UserTuto update failed")
            }
        }
    }


}