package com.dosunsang.dosunsang_server.controller;

import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.dto.UserDto;
import com.dosunsang.dosunsang_server.service.UserService;
import io.swagger.v3.oas.annotations.Operation;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@Slf4j
public class UserController {

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
}