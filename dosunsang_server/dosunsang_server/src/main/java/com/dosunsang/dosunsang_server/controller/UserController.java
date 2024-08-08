package com.dosunsang.dosunsang_server.controller;

import com.amazonaws.services.ec2.model.CertificateAuthenticationRequest;
import com.amazonaws.services.s3.AmazonS3Client;
import com.dosunsang.dosunsang_server.AuthenticationRequest;
import com.dosunsang.dosunsang_server.AuthenticationResponse;
import com.dosunsang.dosunsang_server.JwtUtil;
import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.dto.UserDetailsImpl;
import com.dosunsang.dosunsang_server.dto.UserDto;
import com.dosunsang.dosunsang_server.service.UserService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.security.SecurityRequirement;
import lombok.Getter;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.authentication.AuthenticationManager;
import org.springframework.security.authentication.BadCredentialsException;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.web.bind.annotation.*;


import java.time.Duration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

@RestController
@Slf4j
@SecurityRequirement(name = "bearerAuth")
public class UserController {


    @Value("${cloud.aws.s3.bucket}")
    private String bucketName;

    @Autowired
    private UserService userService;

    @Autowired
    private AuthenticationManager authenticationManager;
    @Autowired
    private JwtUtil jwtUtil;
    @Autowired
    private PasswordEncoder passwordEncoder;

    @PostMapping("/user")
    @Operation(summary = "유저 회원가입", description = "유저의 회원가입입니니다...")
    public ResultDto<UserDto> userAdd(@RequestBody UserDto user) {
        try {
            userService.addUser(user);
            return ResultDto.res(HttpStatus.OK, "성공", user);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping("/user/{userEmail}")
    @Operation(summary = "유저 조회", description = "특정 유저를 조회합니다.")
    public ResultDto<UserDto> getUserInfo(@PathVariable String userEmail) {
        log.info("Received request for user info with email: {}", userEmail);
        try {
            UserDto user = userService.findUser(userEmail);
            return ResultDto.res(HttpStatus.OK, "성공", user);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }


    @PutMapping("/user/{userId}/preferences")
    @Operation(summary = "유저 취향 업데이트", description = "유저의 취향 정보를 업데이트합니다.")
    public ResultDto<UserDto> updateUserPreferences(@PathVariable int userId, @RequestBody List<String> preferences) {
        try {
            boolean updated = userService.updateUserPreferences(userId, preferences);
            if (updated) {
                UserDto updatedUser = userService.findUserId(userId);
                log.info("succeesssssssss");
                return ResultDto.res(HttpStatus.OK, "성공", updatedUser);
            } else {
                return ResultDto.res(HttpStatus.BAD_REQUEST, "업데이트 실패");
            }
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.INTERNAL_SERVER_ERROR, "서버 오류");
        }
    }

    @PutMapping("/user/{userId}/tuto")
    @Operation(summary = "유저 튜토리얼 완료", description = "유저의 튜토리얼 완료 상태를 업데이트합니다")
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
                return ResultDto.res(HttpStatus.OK, "성공!", updatedUser);
            } else {
                return ResultDto.res(HttpStatus.BAD_REQUEST, "업데이트 실패");
            }
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.INTERNAL_SERVER_ERROR, "서버 오류");
        }
    }


    @PostMapping("/authenticate")
    public ResponseEntity<?> createAuthenticationToken(@RequestBody AuthenticationRequest authenticationRequest) throws Exception {
        try {
            authenticationManager.authenticate(
                    new UsernamePasswordAuthenticationToken(authenticationRequest.getUserEmail(), authenticationRequest.getPassword())
            );
        } catch (BadCredentialsException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("Invalid username or password");
        }

        final UserDetailsImpl userDetails = userService.loadUserByEmailJWT(authenticationRequest.getUserEmail());
        final String token = jwtUtil.generateToken(userDetails.getUsername());

        return ResponseEntity.ok(new AuthenticationResponse(token));
    }


    @PostMapping("/register")
    public ResponseEntity<?> registerUser(@RequestBody UserDto user) {
        try {
            UserDto existingUser = userService.findUser(user.getUserEmail());
            if (existingUser != null) {
                return ResponseEntity.badRequest().body(new MessageResponse("User already exists"));
            }

            user.setPassword(passwordEncoder.encode(user.getPassword()));
            boolean success = userService.addUser(user);
            if(success) {
                return ResponseEntity.ok(new MessageResponse("User registered successfully"));
            } else {
                return ResponseEntity.internalServerError().body(new MessageResponse("Failed to register user"));
            }
        } catch (Exception e) {
            log.error("Error during user registration", e);
            return ResponseEntity.internalServerError().body(new MessageResponse("An error occurred during registration"));
        }
    }

    // 새로운 응답 클래스
    public class MessageResponse {
        private String message;

        public MessageResponse(String message) {
            this.message = message;
        }

        public String getMessage() {
            return message;
        }
    }

    @GetMapping("/test")
    public ResponseEntity<String> testApi(){
        return  ResponseEntity.ok("Jwt is....ok");
    }


}