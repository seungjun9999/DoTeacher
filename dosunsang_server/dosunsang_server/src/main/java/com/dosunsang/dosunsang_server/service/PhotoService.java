package com.dosunsang.dosunsang_server.service;

import com.dosunsang.dosunsang_server.dao.PhotoDao;
import com.dosunsang.dosunsang_server.dto.PhotoDto;
import lombok.extern.slf4j.Slf4j;
import org.apache.ibatis.session.SqlSession;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@Slf4j
public class PhotoService {

    @Autowired
    @Qualifier("SessionTemplate")
    private SqlSession sqlSession;

    @Autowired
    private S3Service s3Service;

    public boolean addPhoto(PhotoDto photo) {
        PhotoDao photoDao = sqlSession.getMapper(PhotoDao.class);
        return photoDao.insertPhoto(photo);
    }

    public PhotoDto findPhoto(int photoId) {
        PhotoDao photoDao = sqlSession.getMapper(PhotoDao.class);
        return photoDao.selectPhoto(photoId);
    }

    public List<PhotoDto> findPhotosByUserId(int userId) {
        PhotoDao photoDao = sqlSession.getMapper(PhotoDao.class);
        return photoDao.selectPhotosByUserId(userId);
    }

    public boolean updatePhoto(PhotoDto photo) {
        PhotoDao photoDao = sqlSession.getMapper(PhotoDao.class);
        return photoDao.updatePhoto(photo);
    }

    public boolean deletePhoto(int photoId) {
        PhotoDao photoDao = sqlSession.getMapper(PhotoDao.class);
        return photoDao.deletePhoto(photoId);
    }
}