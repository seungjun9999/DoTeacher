package com.dosunsang.dosunsang_server.controller;

import com.amazonaws.services.s3.AmazonS3Client;
import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.dto.UserDto;
import com.dosunsang.dosunsang_server.service.UserService;
import io.swagger.v3.oas.annotations.Operation;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;
import software.amazon.awssdk.services.s3.S3Client;
import software.amazon.awssdk.services.s3.model.PutObjectRequest;
import software.amazon.awssdk.services.s3.presigner.model.PresignedPutObjectRequest;
import software.amazon.awssdk.services.s3.presigner.model.PutObjectPresignRequest;

import java.time.Duration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

@RestController
@Slf4j
public class UserController {

    @Autowired
    private S3Client s3Client;


    @Value("${cloud.aws.s3.bucket}")
    private String bucketName;

    @Autowired
    private UserService userService;

    @PostMapping("/user")
    @Operation(summary = "유저 회원가입", description = "유저의 회원가입입니다")
    public ResultDto<UserDto> userAdd(@RequestBody UserDto user) {
        try {
            userService.addUser(user);
            return ResultDto.res(HttpStatus.OK, "성공", user);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    // 다른 메서드들은 그대로 유지

    @PutMapping("/user/{userId}/preferences")
    @Operation(summary = "유저 취향 업데이트", description = "유저의 취향 정보를 업데이트합니다.")
    public ResultDto<UserDto> updateUserPreferences(@PathVariable int userId, @RequestBody List<String> preferences) {
        try {
            boolean updated = userService.updateUserPreferences(userId, preferences);
            if (updated) {
                UserDto updatedUser = userService.findUserId(userId);
                return ResultDto.res(HttpStatus.OK, "성공", updatedUser);
            } else {
                return ResultDto.res(HttpStatus.BAD_REQUEST, "업데이트 실패");
            }
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.INTERNAL_SERVER_ERROR, "서버 오류");
        }
    }

    @PutMapping("/user/{userId}/tuto")
    @Operation(summary = "유저 튜토리얼 완료", description = "유저의 튜토리얼 완료 상태를 업데이트합니다.")
    public ResultDto<UserDto> updateUserTutorial(@PathVariable int userId) {
        try {
            boolean updated = userService.updateUserTuto(userId, true);
            if (updated) {
                UserDto updatedUser = userService.findUserId(userId);
                return ResultDto.res(HttpStatus.OK, "성공", updatedUser);
            } else {
                return ResultDto.res(HttpStatus.BAD_REQUEST, "업데이트 실패");
            }
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.INTERNAL_SERVER_ERROR, "서버 오류");
        }
    }

    @PutMapping("/user/{userId}/profile-image")
    @Operation(summary = "유저 프로필 이미지 업데이트", description = "유저의 프로필 이미지 URL을 업데이트합니다.")
    public ResultDto<UserDto> updateUserProfileImage(@PathVariable int userId, @RequestParam String imageUrl) {
        try {
            boolean updated = userService.updateUserProfileImage(userId, imageUrl);
            if (updated) {
                UserDto updatedUser = userService.findUserId(userId);
                return ResultDto.res(HttpStatus.OK, "성공", updatedUser);
            } else {
                return ResultDto.res(HttpStatus.BAD_REQUEST, "업데이트 실패");
            }
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.INTERNAL_SERVER_ERROR, "서버 오류");
        }
    }

    @GetMapping("/user/presigned-url")
    @Operation(summary = "S3 Pre-signed URL 생성", description = "S3에 이미지를 업로드하기 위한 Pre-signed URL을 생성합니다.")
    public ResultDto<Map<String, String>> getPresignedUrl(@RequestParam String fileType) {
        try {
            String key = "profiles/" + UUID.randomUUID() + "." + fileType;
            PutObjectPresignRequest presignRequest = PutObjectPresignRequest.builder()
                    .signatureDuration(Duration.ofMinutes(5))
                    .putObjectRequest(PutObjectRequest.builder()
                            .bucket(bucketName)
                            .key(key)
                            .contentType("image/" + fileType)
                            .build())
                    .build();

            PresignedPutObjectRequest presignedRequest = s3Client.presignPutObject(presignRequest);
            String url = presignedRequest.url().toString();

            Map<String, String> result = new HashMap<>();
            result.put("url", url);
            result.put("key", key);

            return ResultDto.res(HttpStatus.OK, "성공", result);
        } catch (Exception e) {
            log.error("Pre-signed URL 생성 실패", e);
            return ResultDto.res(HttpStatus.INTERNAL_SERVER_ERROR, "Pre-signed URL 생성 실패: " + e.getMessage());
        }
    }
}