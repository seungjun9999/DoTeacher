package com.example.doteacher.ui.gallery

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.palette.graphics.Palette
import com.bumptech.glide.request.transition.Transition
import com.bumptech.glide.Glide
import com.bumptech.glide.load.engine.bitmap_recycle.BitmapPool
import com.bumptech.glide.load.resource.bitmap.BitmapTransformation
import com.bumptech.glide.request.target.SimpleTarget
import com.example.doteacher.R
import com.example.doteacher.databinding.FragmentGalleryBinding
import com.example.doteacher.ui.base.BaseFragment
import com.example.doteacher.ui.gallery.viewmodel.GalleryViewModel
import com.example.doteacher.ui.util.SingletonUtil
import dagger.hilt.android.AndroidEntryPoint
import timber.log.Timber
import java.security.MessageDigest

@AndroidEntryPoint
class GalleryFragment : BaseFragment<FragmentGalleryBinding>(R.layout.fragment_gallery) {
    private val galleryViewModel: GalleryViewModel by viewModels()
    private lateinit var galleryAdapter: GalleryAdapter
    private var isPhotoListActive = true

    override fun initView() {
        initData()
        initAdapter()  // 여기로 이동
        observePhotoData()
        observeUserData()
    }

    override fun onResume() {
        super.onResume()
        loadUserData()
        loadUserPhotos()  // 사진 데이터만 새로 로드
    }

    private fun initData() {
        binding.userData = SingletonUtil.user
        Timber.d(" 지금 갤러리 화면 사앹는? ${SingletonUtil.user}")
    }



    private fun loadUserPhotos() {
        SingletonUtil.user?.id?.let { galleryViewModel.getUserPhotos(it) }
    }

    private fun loadUserData(){
        SingletonUtil.user?.userEmail?.let { email ->
            galleryViewModel.loadUserData(email)
        } ?: run {
            Timber.e("User or user email is null")
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

    private fun observeUserData(){
        galleryViewModel.userData.observe(viewLifecycleOwner){ user ->
            user.let {
                binding.userData = user
                binding.executePendingBindings()
                Timber.d("여기가 전혀 안찍히네 $it")
            }
        }
    }


}