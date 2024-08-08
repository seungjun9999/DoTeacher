package com.dosunsang.dosunsang_server.dto;

import lombok.*;

import java.util.List;

@Getter
@Setter
@ToString
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor(access = AccessLevel.PROTECTED)
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
}