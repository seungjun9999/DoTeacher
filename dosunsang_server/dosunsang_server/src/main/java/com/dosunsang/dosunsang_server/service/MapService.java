package com.dosunsang.dosunsang_server.service;
//gpt-4o-mini

import com.dosunsang.dosunsang_server.dao.MapDao;
import com.dosunsang.dosunsang_server.dto.MapDto;
import org.apache.ibatis.session.SqlSession;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

@Service
public class MapService {
    @Autowired
    @Qualifier("SessionTemplate")
    private SqlSession sqlSession;

    public MapDto getMap(int mapId) {
        MapDao mapDao = sqlSession.getMapper(MapDao.class);
        return mapDao.selectMap(mapId);
    }
}
