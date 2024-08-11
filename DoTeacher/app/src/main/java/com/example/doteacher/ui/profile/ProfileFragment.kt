package com.example.doteacher.ui.profile

import android.animation.Animator
import android.net.Uri
import android.util.Log
import android.view.View
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentProfileBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.main.MainActivity
import com.example.doteacher.ui.profile.viewmodel.ProfileViewModel
import com.example.doteacher.ui.util.S3Uploader
import dagger.hilt.android.AndroidEntryPoint
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext


@AndroidEntryPoint
class ProfileFragment : BaseFragment<FragmentProfileBinding>(R.layout.fragment_profile) {
    private val profileViewModel: ProfileViewModel by viewModels()


    private val getContent = registerForActivityResult(ActivityResultContracts.GetContent()) { uri: Uri? ->
        uri?.let { selectedImageUri ->
            uploadImageToS3(selectedImageUri)
        }
    }

    override fun onResume() {
        super.onResume()
        hideBottomNavigation()
    }

    override fun onPause() {
        super.onPause()
        showBottomNavigation()
    }

    private fun hideBottomNavigation() {
        (activity as? MainActivity)?.hideBottomNavigation()
    }

    private fun showBottomNavigation() {
        (activity as? MainActivity)?.showBottomNavigation()
    }

    override fun initView() {
        initData()
        clickEvent()
        observeViewModel()
    }



    private fun showLoading() {
        binding.imagelottie.visibility = View.VISIBLE
        binding.imagelottie.playAnimation()
        binding.checklottie.visibility = View.GONE
        binding.faillottie.visibility = View.GONE
    }


    private fun hideLoading(isSuccess: Boolean) {
        binding.imagelottie.cancelAnimation()
        binding.imagelottie.visibility = View.GONE

        if (isSuccess) {
            showSuccessAnimation()
        } else {
            showFailAnimation()
        }
    }

    private fun showSuccessAnimation() {
        binding.checklottie.visibility = View.VISIBLE
        binding.checklottie.playAnimation()
        binding.checklottie.addAnimatorListener(object : Animator.AnimatorListener {
            override fun onAnimationStart(animation: Animator) {}
            override fun onAnimationEnd(animation: Animator) {
                binding.checklottie.visibility = View.GONE
            }
            override fun onAnimationCancel(animation: Animator) {}
            override fun onAnimationRepeat(animation: Animator) {}
        })
    }

    private fun showFailAnimation() {
        binding.faillottie.visibility = View.VISIBLE
        binding.faillottie.playAnimation()
        binding.faillottie.addAnimatorListener(object : Animator.AnimatorListener {
            override fun onAnimationStart(animation: Animator) {}
            override fun onAnimationEnd(animation: Animator) {
                binding.faillottie.visibility = View.GONE
            }
            override fun onAnimationCancel(animation: Animator) {}
            override fun onAnimationRepeat(animation: Animator) {}
        })
    }

    private fun initData() {
        binding.userData = profileViewModel.userData.value
    }

    private fun clickEvent() {
        binding.btnProfileToHome.setOnClickListener {
            view?.findNavController()?.popBackStack()
        }

        binding.tvProfileChange.setOnClickListener {
            openImageChooser()
        }
    }

    private fun observeViewModel() {
        profileViewModel.userData.observe(viewLifecycleOwner) { userData ->
            binding.userData = userData
            binding.circleImageView.setImageURI(Uri.parse(userData.userImage))
        }

        profileViewModel.updateResult.observe(viewLifecycleOwner) { event ->
            event.getContentIfNotHandled()?.let { isSuccess ->
                hideLoading(isSuccess)
                if (isSuccess) {
//                    showToast("프로필 이미지가 성공적으로 업데이트되었습니다.")
                } else {
//                    showToast("프로필 이미지 업데이트에 실패했습니다.")
                }
            }
        }
    }

    private fun openImageChooser() {
        getContent.launch("image/*")
    }

    private fun uploadImageToS3(imageUri: Uri) {
        viewLifecycleOwner.lifecycleScope.launch {
            try {
                showLoading()
                val imageUrl = withContext(Dispatchers.IO) {
                    S3Uploader.uploadImage(requireContext(), imageUri)
                }
                updateProfileImage(imageUrl)
            } catch (e: Exception) {
                Log.e("ProfileFragment", "Image upload failed", e)
                hideLoading(false)
            }
        }
    }


    private fun updateProfileImage(imageUrl: String) {
        val userId = profileViewModel.userData.value?.id ?: return
        profileViewModel.updateProfileImage(userId, imageUrl)
    }



}