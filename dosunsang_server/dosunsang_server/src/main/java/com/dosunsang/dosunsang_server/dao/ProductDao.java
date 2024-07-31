package com.dosunsang.dosunsang_server.dao;

import com.dosunsang.dosunsang_server.dto.ProductDto;
import org.apache.ibatis.annotations.Mapper;

import java.util.List;

@Mapper
public interface ProductDao {
    ProductDto selectProduct(int productId);
    List<ProductDto> selectAllProducts();
    List<ProductDto> selectRandomProducts(int count);
}