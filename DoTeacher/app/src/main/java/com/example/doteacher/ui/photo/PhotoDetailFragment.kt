package com.example.doteacher.ui.photo

import android.os.Bundle
import androidx.navigation.fragment.findNavController
import com.example.doteacher.R
import com.example.doteacher.data.model.PhotoData
import com.example.doteacher.databinding.FragmentPhotoDetailBinding
import com.example.doteacher.ui.base.BaseFragment
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class PhotoDetailFragment : BaseFragment<FragmentPhotoDetailBinding>(R.layout.fragment_photo_detail) {

    private var photoData: PhotoData? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
            photoData = it.getParcelable("photoData")
        }
    }

    override fun initView() {
        binding.photoData = photoData
        setupClickListeners()
    }

    private fun setupClickListeners() {
        binding.btnBack.setOnClickListener {
            findNavController().popBackStack()
        }
    }

    companion object {
        fun newInstance(photoData: PhotoData) = PhotoDetailFragment().apply {
            arguments = Bundle().apply {
                putParcelable("photoData", photoData)
            }
        }
    }
}