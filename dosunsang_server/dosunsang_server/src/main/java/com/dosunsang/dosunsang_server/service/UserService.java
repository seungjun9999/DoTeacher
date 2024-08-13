package com.dosunsang.dosunsang_server.service;

import com.dosunsang.dosunsang_server.dao.UserDao;
import com.dosunsang.dosunsang_server.dto.UserDetailsImpl;
import com.dosunsang.dosunsang_server.dto.UserDto;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UserDetailsService;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Optional;

@Service
@Slf4j
public class UserService implements UserDetailsService {

    private final UserDao userDao;

    @Autowired
    public UserService(UserDao userDao) {
        this.userDao = userDao;
    }

    public boolean addUser(UserDto user) {
        return userDao.insertUser(user);
    }

    public UserDto findUser(String userEmail) {
        UserDto user = userDao.selectUser(userEmail);
        if (user != null) {
            log.info("Found user: {}, preferences: {}", user, user.getPreferences());
        } else {
            log.info("User not found for email: {}", userEmail);
        }
        return user;
    }

    public List<UserDto> getUsers() {
        return userDao.selectUsers();
    }

    public UserDto findUserId(int userId) {
        return userDao.selectUserId(userId);
    }

    public boolean updateUserPreferences(int userId, List<String> preferences) {
        return userDao.updateUserPreferences(userId, preferences);
    }

    public boolean updateUserTuto(int userId, boolean userTuto) {
        return userDao.updateUserTuto(userId, userTuto);
    }

    public boolean updateUserDescription(int userId, int userDes){
        return userDao.updateUserDescription(userId,userDes);
    }

    public boolean updateUserProfileImage(int userId, String imageUrl) {
        return userDao.updateUserProfileImage(userId, imageUrl);
    }

    @Override
    public UserDetails loadUserByUsername(String username) throws UsernameNotFoundException {
        UserDto user = findUser(username);
        if (user == null) {
            throw new UsernameNotFoundException("User not found with email: " + username);
        }
        return new UserDetailsImpl(user);
    }

    public UserDetailsImpl loadUserByEmailJWT(String userEmail) throws UsernameNotFoundException {
        UserDto user = findUser(userEmail);
        if (user == null) {
            throw new UsernameNotFoundException("User not found with email: " + userEmail);
        }
        return new UserDetailsImpl(user);
    }

    public boolean updateUserToken(int userId, String token) {
        return userDao.updateUserToken(userId, token);
    }


    public boolean deleteUser(int userId) {
        return userDao.deleteUser(userId);
    }


}