package com.dosunsang.dosunsang_server;


import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class AuthenticationRequest {
    private static final String TAG = AuthenticationRequest.class.getSimpleName();

    private String userEmail;
    private String password;

    public AuthenticationRequest(){}

    public AuthenticationRequest(String userEmail, String password){
        this.userEmail = userEmail;
        this.password = password;
    }
}
