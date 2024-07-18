package com.dosunsang.dosunsang_server.dao;

import com.dosunsang.dosunsang_server.dto.UserDto;
import org.apache.ibatis.annotations.Param;

import java.util.List;

public interface UserDao {

    boolean insertUser(UserDto userDto);

    UserDto selectUser(String userEmail);

    List<UserDto> selectUsers();

    UserDto selectUserId(int userId);

}
