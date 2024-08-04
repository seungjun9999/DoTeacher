package com.example.doteacher.ui.profile

import android.net.Uri
import android.util.Log
import android.widget.Toast
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
                if (isSuccess) {
                    showToast("프로필 이미지가 성공적으로 업데이트되었습니다.")
                } else {
                    showToast("프로필 이미지 업데이트에 실패했습니다.")
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
                val imageUrl = withContext(Dispatchers.IO) {
                    S3Uploader.uploadImage(requireContext(), imageUri)
                }
                updateProfileImage(imageUrl)
            } catch (e: Exception) {
                Log.e("ProfileFragment", "Image upload failed", e)
                showToast("이미지 업로드에 실패했습니다. 다시 시도해주세요.")
            }
        }
    }

    private fun updateProfileImage(imageUrl: String) {
        val userId = profileViewModel.userData.value?.id ?: return
        profileViewModel.updateProfileImage(userId, imageUrl)
    }


    private fun showToast(message: String) {
        Toast.makeText(requireContext(), message, Toast.LENGTH_SHORT).show()
    }
}