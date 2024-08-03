package com.dosunsang.dosunsang_server.dao;

import com.dosunsang.dosunsang_server.dto.UserDto;
import org.apache.ibatis.annotations.Param;

import java.util.List;

public interface UserDao {
    boolean insertUser(UserDto userDto);
    UserDto selectUser(String userEmail);
    List<UserDto> selectUsers();
    UserDto selectUserId(int userId);
    boolean updateUserPreferences(@Param("userId") int userId, @Param("preferences") List<String> preferences);
    boolean updateUserTuto(@Param("userId") int userId, @Param("userTuto") boolean userTuto);
}