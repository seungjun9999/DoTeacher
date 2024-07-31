package com.example.doteacher.ui.gallery

import android.graphics.Color
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentGalleryBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.gallery.viewmodel.GalleryViewModel
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber

@AndroidEntryPoint
class GalleryFragment : BaseFragment<FragmentGalleryBinding>(R.layout.fragment_gallery) {
    private val galleryViewModel: GalleryViewModel by viewModels()
    private lateinit var galleryAdapter: GalleryAdapter
    private var isPhotoListActive = true

    override fun onResume() {
        super.onResume()
        initData()
        initAdapter()
    }

    override fun initView() {
        initData()
        clickEventListener()
        observePhotoData()
    }

    private fun initData() {
        binding.userData = SingletonUtil.user
        loadUserPhotos()
    }

    private fun loadUserPhotos() {
        SingletonUtil.user?.id?.let { galleryViewModel.getUserPhotos(it) }
    }

    private fun clickEventListener() {
        binding.btnReturnHome.setOnClickListener {
            this.findNavController().popBackStack()
        }
        binding.bottomSheetLayout.btnPhotoList.setOnClickListener {
            togglePhotoListBtn()
        }
    }

    private fun togglePhotoListBtn() {
        isPhotoListActive = !isPhotoListActive

        if (isPhotoListActive) {
            binding.bottomSheetLayout.btnPhotoList.setBackgroundResource(R.drawable.gallery_orange_btn)
            binding.bottomSheetLayout.btnPhotoList.setTextColor(Color.parseColor("#f7b84b"))
        } else {
            binding.bottomSheetLayout.btnPhotoList.background = null
            binding.bottomSheetLayout.btnPhotoList.setTextColor(Color.parseColor("#88878a"))
        }
    }

    private fun initAdapter() {
        galleryAdapter = GalleryAdapter()
        binding.bottomSheetLayout.rvMyphotoList.adapter = galleryAdapter

        galleryAdapter.setOnItemClickListener { photoData ->
            this@GalleryFragment.findNavController().navigate(
                R.id.action_galleryFragment_to_photodetailFragment,
                bundleOf("photoData" to photoData)
            )
        }
    }

    private fun observePhotoData() {
        galleryViewModel.userPhotos.observe(viewLifecycleOwner) { photos ->
            Timber.d("Observed photos: ${photos.size}")
            binding.tvPhotoCnt.text = photos.size.toString()
            galleryAdapter.submitList(photos)
        }
    }
}