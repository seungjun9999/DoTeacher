package com.dosunsang.dosunsang_server.service;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.model.*;
import com.drew.imaging.ImageMetadataReader;
import com.drew.metadata.Directory;
import com.drew.metadata.Metadata;
import com.drew.metadata.exif.ExifIFD0Directory;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import javax.imageio.ImageIO;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.*;
import java.net.URLEncoder;
import java.util.Optional;

@Slf4j
@Service
public class S3Service {
    @Autowired
    private AmazonS3 amazonS3;

    @Value("${cloud.aws.s3.bucket}")
    private String bucket;

    private final String DIR_NAME = "profile_images";

    public String upload(String fileName, MultipartFile multipartFile) throws IOException {
        File uploadFile = convertAndRotateImage(multipartFile)
                .orElseThrow(() -> new IllegalArgumentException("MultipartFile -> File 전환 실패"));
        return upload(fileName, uploadFile);
    }

    private String upload(String fileName, File uploadFile) {
        String newFileName = DIR_NAME + "/" + fileName;
        String uploadImageUrl = putS3(uploadFile, newFileName);
        removeNewFile(uploadFile);
        return uploadImageUrl;
    }

    private String putS3(File uploadFile, String fileName) {
        amazonS3.putObject(
                new PutObjectRequest(bucket, fileName, uploadFile)
                        .withCannedAcl(CannedAccessControlList.PublicRead)
        );
        return amazonS3.getUrl(bucket, fileName).toString();
    }

    private void removeNewFile(File targetFile) {
        if (targetFile.delete()) {
            log.info("파일이 삭제되었습니다.");
        } else {
            log.info("파일이 삭제되지 못했습니다.");
        }
    }

    private Optional<File> convertAndRotateImage(MultipartFile file) throws IOException {
        File tempFile = new File("/tmp/upload/" + file.getOriginalFilename());
        file.transferTo(tempFile);

        BufferedImage image = ImageIO.read(tempFile);
        BufferedImage rotatedImage = rotateImageBasedOnExif(image, tempFile);

        File convertFile = new File("/tmp/upload/rotated_" + file.getOriginalFilename());
        if (convertFile.createNewFile()) {
            try (OutputStream os = new FileOutputStream(convertFile)) {
                ImageIO.write(rotatedImage, "jpg", os);
            }
            return Optional.of(convertFile);
        }
        return Optional.empty();
    }

    private BufferedImage rotateImageBasedOnExif(BufferedImage image, File file) {
        try {
            Metadata metadata = ImageMetadataReader.readMetadata(file);
            Directory directory = metadata.getFirstDirectoryOfType(ExifIFD0Directory.class);
            int orientation = directory.getInt(ExifIFD0Directory.TAG_ORIENTATION);

            double angle = 0;
            switch (orientation) {
                case 6: // 90 degrees CW
                    angle = Math.toRadians(90);
                    break;
                case 3: // 180 degrees
                    angle = Math.toRadians(180);
                    break;
                case 8: // 90 degrees CCW
                    angle = Math.toRadians(270);
                    break;
                default:
                    break;
            }

            if (angle != 0) {
                AffineTransform transform = new AffineTransform();
                transform.rotate(angle, image.getWidth() / 2.0, image.getHeight() / 2.0);
                AffineTransformOp op = new AffineTransformOp(transform, AffineTransformOp.TYPE_BILINEAR);
                image = op.filter(image, null);
            }
        } catch (Exception e) {
            log.error("EXIF 정보를 읽는 중 오류 발생: ", e);
        }
        return image;
    }

    public ResponseEntity<byte[]> download(String fileName) throws IOException {
        S3Object awsS3Object = amazonS3.getObject(new GetObjectRequest(bucket, DIR_NAME + "/" + fileName));
        S3ObjectInputStream s3is = awsS3Object.getObjectContent();
        byte[] bytes = s3is.readAllBytes();

        String downloadedFileName = URLEncoder.encode(fileName, "UTF-8").replace("+", "%20");
        HttpHeaders httpHeaders = new HttpHeaders();
        httpHeaders.setContentType(MediaType.IMAGE_JPEG);
        httpHeaders.setContentLength(bytes.length);
        httpHeaders.setContentDispositionFormData("attachment", downloadedFileName);
        return new ResponseEntity<>(bytes, httpHeaders, HttpStatus.OK);
    }
}
