package com.dosunsang.dosunsang_server.dto;


import lombok.Data;

@Data
public class ProductDto {
    int productId;
    String productName;
    String productDesc;
    String productUrl;
    String productWriter;
    int pos_x;
    int pos_y;
}
