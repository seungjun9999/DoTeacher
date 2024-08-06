package com.dosunsang.dosunsang_server.dto;

import lombok.Data;
import java.time.LocalDateTime;

@Data
public class PhotoDto {
    private int photoId;
    private String fileName;
    private String description;
    private String imageUrl;
    private int userId;
    private LocalDateTime createdAt;
}