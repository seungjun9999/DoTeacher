SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0;
SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0;
SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';
CREATE SCHEMA IF NOT EXISTS dosunsangdb DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
USE dosunsangdb;
-- 기존 테이블 삭제
DROP TABLE IF EXISTS user_preference;
DROP TABLE IF EXISTS preference;
DROP TABLE IF EXISTS photo;
DROP TABLE IF EXISTS product;
DROP TABLE IF EXISTS user;
-- user 테이블 생성
CREATE TABLE IF NOT EXISTS user (
    id INT AUTO_INCREMENT NOT NULL,
    useremail VARCHAR(50) NOT NULL,
    username VARCHAR(16) NOT NULL,
    userImage VARCHAR(200) NOT NULL,
    PRIMARY KEY (id)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;
-- photo 테이블 생성
CREATE TABLE IF NOT EXISTS photo (
    photo_id INT AUTO_INCREMENT NOT NULL,
    file_name VARCHAR(255) NOT NULL,
    description TEXT,
    image_url VARCHAR(2083) NOT NULL,
    user_id INT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (photo_id),
    FOREIGN KEY (user_id) REFERENCES user (id) ON DELETE SET NULL
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;
-- product 테이블 생성
CREATE TABLE IF NOT EXISTS `product` (
    `product_id` INT AUTO_INCREMENT NOT NULL,
    `product_name` VARCHAR(255) NOT NULL,
    `description` TEXT,
    `product_url` VARCHAR(2083) NOT NULL,
    PRIMARY KEY (`product_id`)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;
-- preference 테이블 생성
CREATE TABLE IF NOT EXISTS preference (
    preference_id INT AUTO_INCREMENT NOT NULL,
    preference_name VARCHAR(50) NOT NULL,
    PRIMARY KEY (preference_id)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;
-- user_preference 테이블 생성
CREATE TABLE IF NOT EXISTS user_preference (
    user_id INT NOT NULL,
    preference_id INT NOT NULL,
    PRIMARY KEY (user_id, preference_id),
    FOREIGN KEY (user_id) REFERENCES user (id) ON DELETE CASCADE,
    FOREIGN KEY (preference_id) REFERENCES preference (preference_id) ON DELETE CASCADE
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;
-- 인덱스 추가
CREATE INDEX idx_user_id ON photo (user_id);
CREATE INDEX idx_preference_id ON user_preference (preference_id);
ALTER TABLE user ADD COLUMN preferences TEXT;
SET SQL_MODE=@OLD_SQL_MODE;
SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS;
SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS;
-- 테이블 생성 확인
SHOW TABLES;
-- 테이블 구조 확인
DESCRIBE user;
DESCRIBE photo;
DESCRIBE preference;
DESCRIBE user_preference;
-- 데이터 확인 (선택사항)
SELECT * FROM dosunsangdb.user;
SELECT * FROM dosunsangdb.photo;
SELECT * FROM dosunsangdb.product;
SELECT * FROM dosunsangdb.preference;
SELECT * FROM dosunsangdb.user_preference;