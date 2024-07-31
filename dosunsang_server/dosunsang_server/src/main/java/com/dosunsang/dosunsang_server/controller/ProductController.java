package com.dosunsang.dosunsang_server.controller;

import com.dosunsang.dosunsang_server.dto.ProductDto;
import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.service.ProductService;
import io.swagger.v3.oas.annotations.Operation;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@Slf4j
@RequestMapping("/products")
public class ProductController {
    @Autowired
    private ProductService productService;

    @GetMapping("")
    @Operation(summary = "전체 작품 조회", description = "전체 작품을 조회합니다")
    public ResultDto<List<ProductDto>> getAllProducts() {
        try {
            List<ProductDto> products = productService.getProducts();
            return ResultDto.res(HttpStatus.OK, "성공", products);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping("/{productId}")
    @Operation(summary = "특정 작품 조회", description = "특정 작품을 조회합니다")
    public ResultDto<ProductDto> getProduct(@PathVariable int productId) {
        try {
            ProductDto productDto = productService.getProduct(productId);
            return ResultDto.res(HttpStatus.OK, "성공", productDto);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @GetMapping("/random")
    @Operation(summary = "랜덤 작품 10개 조회", description = "랜덤으로 10개의 작품을 조회합니다")
    public ResultDto<List<ProductDto>> getRandomProducts() {
        try {
            List<ProductDto> randomProducts = productService.getRandomProducts(10);
            return ResultDto.res(HttpStatus.OK, "성공", randomProducts);
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

}
