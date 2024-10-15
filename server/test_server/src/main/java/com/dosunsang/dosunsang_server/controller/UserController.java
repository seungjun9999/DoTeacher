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
    @Operation(summary = "유저 회원가입", description = "유저의 회원가입")
    public ResultDto<UserDto> userAdd(@RequestBody UserDto user) {
        try {
            log.info(user.toString());
            //userService.addUser(user);
            return ResultDto.res(HttpStatus.OK, "성공", user);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping("/user")
    @Operation(summary = "유저 조회", description = "유저의 정보를 조회합니다.")
    public ResultDto<UserDto> getUser(@RequestParam(value="userEmail") String userEmail) {
        try {
            UserDto userDto = userService.findUser(userEmail);
            return ResultDto.res(HttpStatus.OK, "성공", userDto);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping("/users")
    @Operation(summary = "전체 유저 조회", description = "전체 유저의 정보를 조회합니다.")
    public ResultDto<List<UserDto>> getUser() {
        try {
            List<UserDto> userDto = userService.getUsers() ;
            return ResultDto.res(HttpStatus.OK, "성공", userDto);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

}
