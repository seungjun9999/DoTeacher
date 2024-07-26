package com.dosunsang.dosunsang_server.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import org.springframework.http.HttpStatus;

@Data
@Builder
@AllArgsConstructor
public class ResultDto<T> {
    private int statusCode;
    private String msg;
    private T data;

    public ResultDto(final HttpStatus statusCode, final String msg) {
        this.statusCode = statusCode.value();
        this.msg = msg;
        this.data = null;
    }

    public static <T> ResultDto<T> res(final HttpStatus statusCode, final String msg) {
        return res(statusCode, msg, null);
    }

    public static <T> ResultDto<T> res(final HttpStatus statusCode, final String msg, final T t) {
        return ResultDto.<T>builder()
                .data(t)
                .statusCode(statusCode.value())
                .msg(msg)
                .build();
    }
}

