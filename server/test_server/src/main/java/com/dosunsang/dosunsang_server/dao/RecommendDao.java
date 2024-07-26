package com.dosunsang.dosunsang_server.dao;

import com.dosunsang.dosunsang_server.dto.ItemDto;


import java.util.List;

public interface RecommendDao {

    List<ItemDto> RecommendItem();
    List<ItemDto> RecommendPath();
}
