package com.dosunsang.dosunsang_server.dao;

import com.dosunsang.dosunsang_server.dto.UserDto;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;
import java.util.List;
import java.util.Optional;

@Mapper
public interface UserDao{
    boolean insertUser(UserDto userDto);
    UserDto selectUser(String userEmail);
    List<UserDto> selectUsers();
    UserDto selectUserId(int userId);
    boolean updateUserPreferences(@Param("userId") int userId, @Param("preferences") List<String> preferences);
    boolean updateUserTuto(@Param("userId") int userId, @Param("userTuto") boolean userTuto);
    boolean updateUserProfileImage(@Param("userId") int userId, @Param("userImage") String userImage);
    boolean updateUserToken(int userId, String token);
    boolean deleteUser(int userId);
}