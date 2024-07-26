package com.dosunsang.dosunsang_server.dto;

import lombok.Getter;
import lombok.ToString;

import java.util.ArrayList;

@Getter
@ToString
public class UserDto {

    public int id;
    public String userEmail;
    public String userName;
    public String userImage;
}
