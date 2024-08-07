package com.dosunsang.dosunsang_server.controller;

import com.dosunsang.dosunsang_server.dto.PhotoDto;
import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.service.PhotoService;
import com.dosunsang.dosunsang_server.service.S3Service;
import io.swagger.v3.oas.annotations.Operation;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.util.List;

@RestController
@Slf4j
public class PhotoController {

    @Autowired
    private S3Service s3Service;

    @Autowired
    private PhotoService photoService;

    @PostMapping(path = "/photo", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    @Operation(summary = "사진 업로드", description = "사진을 DB에 업로드 합니다!")
    public ResultDto<PhotoDto> uploadPhoto(
            @RequestParam("file") MultipartFile file,
            @RequestParam("description") String description,
            @RequestParam("userId") int userId
    ) {
        try {
            String fileName = file.getOriginalFilename();
            String extend = fileName.substring(fileName.lastIndexOf("."));
            String imageUrl = s3Service.upload(fileName, file, extend);

            PhotoDto photoDto = new PhotoDto();
            photoDto.setFileName(fileName);
            photoDto.setDescription(description);
            photoDto.setImageUrl(imageUrl);
            photoDto.setUserId(userId);

            photoService.addPhoto(photoDto);

            return ResultDto.res(HttpStatus.OK, "성공", photoDto);
        } catch (Exception e) {
            log.error("Photo upload failed", e);
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패: " + e.getMessage());
        }
    }

    @GetMapping(path = "/photo/{photoId}")
    @Operation(summary = "사진 조회", description = "특정 사진을 조회합니다")
    public ResultDto<PhotoDto> getPhoto(@PathVariable int photoId) {
        try {
            PhotoDto photoDto = photoService.findPhoto(photoId);
            return ResultDto.res(HttpStatus.OK, "성공", photoDto);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping(path = "/photos")
    @Operation(summary = "전체 사진 조회", description = "전체 사진을 조회합니다")
    public ResultDto<List<PhotoDto>> getAllPhotos() {
        try {
            List<PhotoDto> photos = photoService.findAllPhotos();
            return ResultDto.res(HttpStatus.OK, "성공", photos);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }
    @GetMapping("/photos/{userId}")
    public ResultDto<List<PhotoDto>> getUserPhotos(@PathVariable int userId) {
        try {
            List<PhotoDto> photos = photoService.findPhotosByUserId(userId);
            return ResultDto.res(HttpStatus.OK, "성공", photos);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패: " + e.getMessage());
        }
    }

    @GetMapping(path = "/photo/download/{fileName}")
    @Operation(summary = "사진 다운로드", description = "사진을 다운로드 합니다")
    public ResponseEntity<byte[]> downloadPhoto(@PathVariable String fileName) throws IOException {
        return s3Service.download(fileName);
    }
}