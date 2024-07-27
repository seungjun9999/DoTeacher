package com.dosunsang.dosunsang_server.controller;

import com.dosunsang.dosunsang_server.dto.PhotoDto;
import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.service.PhotoService;
import com.dosunsang.dosunsang_server.service.S3Service;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;

@RestController
@Slf4j
public class S3Controller {

    @Autowired
    private S3Service s3Service;

    @Autowired
    private PhotoService photoService;

    @PostMapping(path = "/photo", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    public ResultDto<PhotoDto> uploadPhoto(
            @RequestParam("file") MultipartFile file,
            @RequestParam("description") String description,
            @RequestParam("userId") int userId  // userId를 파라미터로 추가
    ) {
        try {
            String fileName = file.getOriginalFilename();
            String extend = fileName.substring(fileName.lastIndexOf("."));
            String imageUrl = s3Service.upload(fileName, file, extend);

            PhotoDto photoDto = new PhotoDto();
            photoDto.setFileName(fileName);
            photoDto.setDescription(description);
            photoDto.setImageUrl(imageUrl);
            photoDto.setUserId(userId);  // userId 설정

            photoService.addPhoto(photoDto);

            return ResultDto.res(HttpStatus.OK, "성공", photoDto);
        } catch (Exception e) {
            log.error("Photo upload failed", e);
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패: " + e.getMessage());
        }
    }

    @GetMapping(path = "/photo/{photoId}")
    public ResultDto<PhotoDto> getPhoto(@PathVariable int photoId) {
        try {
            PhotoDto photoDto = photoService.findPhoto(photoId);
            return ResultDto.res(HttpStatus.OK, "성공", photoDto);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping(path = "/photo/download/{fileName}")
    public ResponseEntity<byte[]> downloadPhoto(@PathVariable String fileName) throws IOException {
        return s3Service.download(fileName);
    }
}