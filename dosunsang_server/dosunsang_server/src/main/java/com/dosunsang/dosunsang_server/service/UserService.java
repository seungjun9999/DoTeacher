package com.dosunsang.dosunsang_server.service;

import com.dosunsang.dosunsang_server.dao.UserDao;
import com.dosunsang.dosunsang_server.dto.UserDto;
import lombok.extern.slf4j.Slf4j;
import org.apache.ibatis.session.SqlSession;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@Slf4j
public class UserService {

    @Autowired
    @Qualifier("SessionTemplate")
    private SqlSession sqlSession;

    public boolean addUser(UserDto user) {
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.insertUser(user);
    }

    public UserDto findUser(String userEmail) {
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.selectUser(userEmail);
    }

    public List<UserDto> getUsers() {
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.selectUsers();
    }

    public UserDto findUserId(int userId){
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.selectUserId(userId);
    }

    public boolean updateUserPreferences(int userId, List<String> preferences) {
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.updateUserPreferences(userId, preferences);
    }

    public boolean updateUserTuto(int userId, boolean userTuto) {
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.updateUserTuto(userId, userTuto);
    }

    public boolean updateUserProfileImage(int userId, String imageUrl) {
        UserDao userDao = sqlSession.getMapper(UserDao.class);
        return userDao.updateUserProfileImage(userId, imageUrl);
    }
}