package com.dosunsang.dosunsang_server.dao;

import com.dosunsang.dosunsang_server.dto.PhotoDto;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

import java.util.List;

@Mapper
public interface PhotoDao {
    boolean insertPhoto(PhotoDto photo);
    PhotoDto selectPhoto(int photoId);
    List<PhotoDto> selectPhotosByUserId(int userId);
    boolean updatePhoto(PhotoDto photo);
    boolean deletePhoto(int photoId);
}