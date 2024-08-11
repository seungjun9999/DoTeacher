package com.example.doteacher.ui.home

import android.animation.Animator
import android.net.Uri
import android.view.View
import androidx.fragment.app.viewModels
import androidx.navigation.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import com.bumptech.glide.Glide
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

        observeData()
        clickEventListener()
        checkAndShowDialog()
        initAdapter()
        observeUserTutoUpdate()
        loadProfileImage()
        observeLoading()
    }
    override fun onResume() {
        super.onResume()
        loadUserData()
        homeViewModel.getRandomProducts()
    }

    private fun initData() {
        binding.userData = SingletonUtil.user
        binding.executePendingBindings()
        Timber.d("Home screen user data: ${SingletonUtil.user}")
    }

    private fun loadProfileImage() {
        SingletonUtil.user?.userImage?.let { imageUrl ->
            if (imageUrl.startsWith("content://") || imageUrl.startsWith("file://")) {
                binding.imgProfile.setImageURI(Uri.parse(imageUrl))
            } else {
                Glide.with(this)
                    .load(imageUrl)
                    .into(binding.imgProfile)
            }
        }
    }

    private fun loadUserData() {
        SingletonUtil.user?.userEmail?.let { email ->
            homeViewModel.loadUserData(email)
        } ?: run {
            Timber.e("User email is null in HomeFragment")
        }
    }

    private fun clickEventListener() {
        binding.btnViewall.setOnClickListener {
            view?.findNavController()?.navigate(R.id.action_mainFragment_to_productFragment)
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



    private fun observeLoading() {
        homeViewModel.isLoading.observe(viewLifecycleOwner) { isLoading ->
            if (isLoading) {
                binding.startlottie.visibility = View.VISIBLE
                binding.productRecycle.visibility = View.GONE
                binding.startlottie.playAnimation()
            } else {
                playFinalAnimation()
            }
        }
    }

    private fun playFinalAnimation() {
        binding.startlottie.addAnimatorListener(object : Animator.AnimatorListener {
            override fun onAnimationStart(animation: Animator) {}

            override fun onAnimationEnd(animation: Animator) {
                binding.startlottie.removeAllAnimatorListeners()
                binding.startlottie.visibility = View.GONE
                binding.productRecycle.visibility = View.VISIBLE
                binding.startlottie.cancelAnimation()
            }

            override fun onAnimationCancel(animation: Animator) {}

            override fun onAnimationRepeat(animation: Animator) {}
        })
        binding.startlottie.speed = 2.0f
        binding.startlottie.playAnimation()
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

        homeViewModel.userData.observe(viewLifecycleOwner) { userData ->
            binding.userData = userData
            loadProfileImage()
        }
    }

    private fun observeUserTutoUpdate() {
        homeViewModel.userTutoUpdated.observe(viewLifecycleOwner) { updated ->
            if (updated) {
                Timber.d("UserTuto updated successfully")
            } else {
                Timber.d("UserTuto update failed")
            }
        }
    }


}