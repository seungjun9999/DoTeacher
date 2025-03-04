package com.dosunsang.dosunsang_server.dto;

import lombok.*;

import java.util.List;

@Getter
@Setter
@ToString
public class UserDto {
    public int id;
    public String userEmail;
    public String userName;
    public String password;
    public String userImage;
    public String token;
    public boolean userTuto;
    public boolean prefSelect;
    public List<String> preferences;
    public int userDes;
}