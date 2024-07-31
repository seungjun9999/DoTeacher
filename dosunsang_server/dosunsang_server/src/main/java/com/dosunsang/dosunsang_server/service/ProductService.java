package com.dosunsang.dosunsang_server.service;
//gpt-4o-mini

import com.dosunsang.dosunsang_server.dao.ProductDao;
import com.dosunsang.dosunsang_server.dto.ProductDto;
import lombok.extern.slf4j.Slf4j;
import org.apache.ibatis.session.SqlSession;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@Slf4j
public class ProductService {


    @Autowired
    @Qualifier("SessionTemplate")
    private SqlSession sqlSession;

    public List<ProductDto> getProducts() {
        ProductDao productDao = sqlSession.getMapper(ProductDao.class);
        return productDao.selectAllProducts();
    }

    public ProductDto getProduct(int productId) {
        ProductDao productDao = sqlSession.getMapper(ProductDao.class);
        return productDao.selectProduct(productId);
    }

    public List<ProductDto> getRandomProducts(int count){
        ProductDao productDao = sqlSession.getMapper(ProductDao.class);
        List<ProductDto> products = productDao.selectRandomProducts(count);
        log.info("Random products retrieved: {}", products);
        return products;
    }

}
