package com.dosunsang.dosunsang_server.dao;
import com.dosunsang.dosunsang_server.dto.MapDto;
import org.apache.ibatis.annotations.Mapper;

@Mapper
public interface MapDao {
    //boolean insertMap(MapDto photo); map은 직접 db에 넣을듯
    MapDto selectMap(int mapId);
}