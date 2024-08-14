SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0;
SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0;
SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION';

CREATE SCHEMA IF NOT EXISTS `dosunsangdb` DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;
USE `dosunsangdb`;

-- 기존 테이블 삭제
DROP TABLE IF EXISTS user_preference;
DROP TABLE IF EXISTS preference;
DROP TABLE IF EXISTS photo;
DROP TABLE IF EXISTS user;
DROP TABLE IF EXISTS product;

-- user 테이블 생성
CREATE TABLE IF NOT EXISTS `user` (
    `id` INT AUTO_INCREMENT NOT NULL,
    `useremail` VARCHAR(50) NOT NULL,
    `password` VARCHAR(255),
    `username` VARCHAR(16) NOT NULL,
    `userImage` VARCHAR(200) NOT NULL,
    `token` VARCHAR(3000),
    `userTuto` BOOLEAN DEFAULT FALSE,
    `prefSelect` BOOLEAN DEFAULT FALSE,
    `preferences` TEXT,
    `userDes` INT,
    PRIMARY KEY (`id`)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;

-- photo 테이블 생성
CREATE TABLE IF NOT EXISTS `photo` (
    `photo_id` INT AUTO_INCREMENT NOT NULL,
    `file_name` VARCHAR(255) NOT NULL,
    `description` TEXT,
    `image_url` VARCHAR(2083) NOT NULL,
    `user_id` INT,
    `created_at` TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (`photo_id`),
    FOREIGN KEY (`user_id`) REFERENCES `user` (`id`) ON DELETE SET NULL
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;

-- preference 테이블 생성
CREATE TABLE IF NOT EXISTS `preference` (
    `preference_id` INT AUTO_INCREMENT NOT NULL,
    `preference_name` VARCHAR(50) NOT NULL,
    PRIMARY KEY (`preference_id`)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;

-- user_preference 테이블 생성
CREATE TABLE IF NOT EXISTS `user_preference` (
    `user_id` INT NOT NULL,
    `preference_id` INT NOT NULL,
    PRIMARY KEY (`user_id`, `preference_id`),
    FOREIGN KEY (`user_id`) REFERENCES `user` (`id`) ON DELETE CASCADE,
    FOREIGN KEY (`preference_id`) REFERENCES `preference` (`preference_id`) ON DELETE CASCADE
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;

-- product 테이블 생성
CREATE TABLE IF NOT EXISTS `product` (
    `product_id` INT AUTO_INCREMENT NOT NULL,
    `product_name` VARCHAR(255) NOT NULL,
    `description` TEXT,
    `product_url` VARCHAR(2083) NOT NULL,
    `product_writer` VARCHAR(255) NOT NULL,
    PRIMARY KEY (`product_id`)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COLLATE = utf8mb4_unicode_ci;


-- 인덱스 추가
CREATE INDEX `idx_user_id` ON `photo` (`user_id`);
CREATE INDEX `idx_product_name` ON `product` (`product_name`);
CREATE INDEX `idx_preference_id` ON `user_preference` (`preference_id`);

SET SQL_MODE=@OLD_SQL_MODE;
SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS;
SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS;

-- 테이블 생성 확인
SHOW TABLES;

-- 테이블 구조 확인
DESCRIBE `user`;
DESCRIBE `photo`;
DESCRIBE `preference`;
DESCRIBE `user_preference`;

-- 데이터 확인 (선택사항)
SELECT * FROM dosunsangdb. user;
SELECT * FROM dosunsangdb.photo;
SELECT * FROM dosunsangdb.product;
SELECT * FROM dosunsangdb.preference;
SELECT * FROM dosunsangdb.user_preference;
select * from user;

DELETE FROM photo WHERE photo_id > 0;


INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("난맹첩", "김정희", "난맹첩은 김정희의 대표작 중 하나로, 난초를 주제로 한 일련의 그림들을 모은 화첩입니다. 각 페이지마다 다양한 구도와 필법으로 난초를 표현했습니다. 김정희는 특유의 갈필(渴筆) 기법을 사용하여 난초의 잎과 꽃을 표현했는데, 이는 마른 붓으로 그리는 기법으로 강한 생명력과 기개를 나타냅니다. 작품에는 대부분 시나 제발(題跋)이 함께 쓰여 있어, 그림과 글이 조화를 이루는 문인화의 전통을 잘 보여줍니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EA%B9%80%EC%A0%95%ED%9D%AC_%EB%82%9C%EB%A7%B9%EC%B2%A9.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("아홉번째 파도", "이반 아이바조프스키", "이 작품은 러시아 낭만주의 해양화의 대표작입니다. 거대한 파도가 난파선의 생존자들을 위협하는 장면을 극적으로 묘사했습니다. '아홉번째 파도'라는 제목은 민간 전설에서 아홉번째 파도가 가장 강력하고 위험하다는 믿음에서 왔습니다. 아이바조프스키는 빛의 효과를 탁월하게 사용하여 파도의 투명함과 힘을 표현했으며, 하늘의 밝은 부분은 희망을 상징합니다. 이 작품은 인간의 의지와 자연의 힘 사이의 대결을 상징적으로 보여줍니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%9D%B4%EB%B0%98+%EC%95%84%EC%9D%B4%EB%B0%94%EC%A1%B0%ED%94%84%EC%8A%A4%ED%82%A4_%EC%95%84%ED%99%89%EB%B2%88%EC%A7%B8+%ED%8C%8C%EB%8F%84.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("부유세계 마타베이의 걸작", "우타가와 구니요시", "이 작품은 일본 우키요에의 대표적인 예로, 전설적인 화가 이와사 마타베이의 이야기를 담고 있습니다. 구니요시는 복잡한 구도와 생생한 색채를 사용하여 환상적이고 초현실적인 장면을 만들어냈습니다. 작품에는 일본 전통 신화와 민담의 요소들이 가득하며, 섬세한 선의 사용과 정교한 패턴이 특징적입니다. 이 작품은 우키요에 예술의 정점을 보여주는 동시에, 당시 일본 대중문화의 풍부함을 반영합니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%9A%B0%ED%83%80%EA%B0%80%EC%99%80+%EC%BF%A0%EB%8B%88%EC%9A%94%EC%8B%9C_%EB%B6%80%EC%9C%A0%EC%84%B8%EA%B3%84+%EB%A7%88%ED%83%80%EB%B2%A0%EC%9D%B4%EC%9D%98+%EA%B1%B8%EC%9E%91%EC%9D%98+%EA%B8%B0%EC%A0%81.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("겨울 풍경", "카밀 피사로", "피사로의 '겨울 풍경'은 프랑스 시골의 겨울 풍경을 묘사한 작품입니다. 인상주의 특유의 빛과 색채 표현이 돋보이며, 눈 덮인 들판과 마을의 평화로운 분위기를 섬세하게 포착했습니다. 피사로는 부드러운 붓터치와 미묘한 색조의 변화를 통해 겨울 대기의 차가움과 동시에 따뜻함을 표현했습니다. 하늘과 눈의 색채 대비, 원근법의 사용 등을 통해 깊이감과 공간감을 효과적으로 나타냈습니다. 이 작품은 일상적인 농촌 풍경을 시적이고 아름답게 표현한 피사로의 능력을 잘 보여줍니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%B9%B4%EB%B0%80+%ED%94%BC%EC%82%AC%EB%A1%9C_%EB%8C%80%EB%A1%9C%2C%EB%88%88%EC%9D%98+%ED%9A%A8%EA%B3%BC.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("게르니카", "파블로 피카소", "'게르니카'는 스페인 내전 중 독일 나치의 폭격으로 파괴된 바스크 마을 게르니카의 비극을 담은 대형 벽화입니다. 피카소는 큐비즘적 요소와 강렬한 흑백 대비를 사용하여 전쟁의 공포와 고통을 표현했습니다. 작품 속 왜곡된 인물과 동물들(소, 말, 비둘기 등)은 각각 상징적 의미를 지니고 있으며, 중앙의 전구는 진실과 희망을 상징합니다. 이 작품은 20세기 가장 영향력 있는 반전 예술품 중 하나로 평가받으며, 전쟁의 잔혹함과 무의미함을 강력하게 전달합니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%ED%8C%8C%EB%B8%94%EB%A1%9C+%ED%94%BC%EC%B9%B4%EC%86%8C_%EA%B2%8C%EB%A5%B4%EB%8B%88%EC%B9%B4.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("무제", "잭슨 폴록", "잭슨 폴록의 '무제' 작품들은 그의 독특한 드립 페인팅 기법을 잘 보여줍니다. 폴록은 캔버스를 바닥에 놓고 그 위를 걸어다니며 페인트를 흘리고, 튀기고, 붓는 방식으로 작업했습니다. 이 기법은 우연성과 즉흥성을 강조하며, 작가의 신체적 움직임이 그대로 작품에 반영됩니다. 복잡하게 얽힌 선들과 색채의 층들은 강렬한 에너지와 리듬감을 전달하며, 관객으로 하여금 자유로운 해석을 할 수 있게 합니다. 이러한 작품들은 전통적인 구상미술의 개념을 완전히 탈피한 새로운 형태의 예술을 제시했다는 점에서 현대미술사에 큰 영향을 미쳤습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%9E%AD%EC%8A%A8+%ED%8F%B4%EB%A1%9D_%EB%AC%B4%EC%A0%9C.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("송하음다도", "심사정", "'송하음다도'는 '소나무 아래에서 차를 마시다'라는 뜻으로, 문인들의 이상적인 삶을 표현한 작품입니다. 심사정은 섬세한 필치로 소나무의 질감과 형태를 생생하게 표현했으며, 차를 마시는 인물들의 모습을 통해 여유로운 분위기를 연출했습니다. 화면 구성에 있어 근경, 중경, 원경의 배치가 조화롭게 이루어져 깊이감을 더했습니다. 또한, 여백의 미를 활용하여 한적하고 고요한 분위기를 효과적으로 표현했습니다. 이 작품은 조선 후기 문인화의 특징을 잘 보여주는 동시에, 심사정 특유의 섬세하고 정교한 화풍을 대표합니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%8B%AC%EC%82%AC%EC%A0%95_%EC%86%A1%ED%95%98%EC%9D%8C%EB%8B%A4%EB%8F%84..jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("기억의 지속", "살바도르 달리", "'기억의 지속'은 달리의 가장 유명한 작품 중 하나로, 녹아내리는 시계들이 특징적입니다. 이 작품은 시간의 유동성과 주관성을 표현하고 있으며, 프로이트의 정신분석학 이론에 영향을 받았습니다. 바위, 나무, 해변 등의 배경은 달리의 고향 카탈로니아 지역을 연상시킵니다. 중앙의 기괴한 형태는 달리 자신의 얼굴을 왜곡한 것으로 해석되기도 합니다. 이 작품은 시간, 기억, 무의식에 대한 달리의 독특한 시각을 보여주며, 초현실주의 미술의 대표작으로 손꼽힙니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%82%B4%EB%B0%94%EB%8F%84%EB%A5%B4+%EB%8B%AC%EB%A6%AC_%EA%B8%B0%EC%96%B5%EC%9D%98+%EC%A7%80%EC%86%8D_.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("밤의 카페 테라스", "빈센트 반 고흐", "이 작품은 고흐가 프랑스 아를에서 그린 것으로, 밤의 카페 테라스를 생생하게 묘사했습니다. 강렬한 노란색과 푸른색의 대비가 특징적이며, 이는 고흐의 색채 사용의 대표적 예입니다. 카페의 따뜻한 불빛과 별이 빛나는 하늘의 대조는 밤의 활기찬 분위기를 효과적으로 전달합니다. 고흐는 원근법과 명암을 과장하여 사용함으로써 장면에 더욱 극적인 효과를 주었습니다. 이 작품은 고흐의 후기 화풍을 잘 보여주며, 그의 정서적 상태와 주변 세계에 대한 독특한 시각을 반영합니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EB%B9%88%EC%84%BC%ED%8A%B8+%EB%B0%98+%EA%B3%A0%ED%9D%90_%EB%B0%A4%EC%9D%98+%EC%B9%B4%ED%8E%98+%ED%85%8C%EB%9D%BC%EC%8A%A4.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("별이 빛나는 밤", "빈센트 반 고흐", "'별이 빛나는 밤'은 고흐의 가장 유명한 작품 중 하나입니다. 소용돌이치는 하늘과 밝게 빛나는 별들이 특징적이며, 이는 고흐의 내면 세계와 자연에 대한 독특한 해석을 보여줍니다. 격정적인 붓질과 강렬한 색채 대비는 고흐의 감정적 상태를 반영합니다. 중앙의 사이프러스 나무는 하늘과 땅을 연결하는 듯한 모습으로 묘사되었으며, 마을의 평화로운 모습과 대조를 이룹니다. 이 작품은 고흐가 정신병원에 입원해 있을 때 그린 것으로, 그의 정신적 고뇌와 예술적 비전이 융합된 결과물로 여겨집니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EB%B9%88%EC%84%BC%ED%8A%B8+%EB%B0%98+%EA%B3%A0%ED%9D%90_%EB%B3%84%EC%9D%B4+%EB%B9%9B%EB%82%98%EB%8A%94+%EB%B0%A4.jpg");


INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("서당", "김홍도", "'서당'은 김홍도의 대표적인 풍속화 중 하나입니다. 이 작품은 조선 시대 서당의 일상을 생생하게 묘사하고 있습니다. 화면 중앙에는 훈장이 위엄 있게 앉아 있고, 그 주변으로 다양한 연령대의 학생들이 각자의 모습으로 공부하고 있습니다. 김홍도는 각 인물의 표정과 자세를 섬세하게 포착하여, 학업에 열중한 학생, 장난치는 학생, 졸고 있는 학생 등 다양한 모습을 사실적으로 표현했습니다. 이 작품은 당시의 교육 현장을 생생하게 전달할 뿐만 아니라, 인간 본성의 다양한 면모를 유머러스하게 드러내고 있습니다. 김홍도의 뛰어난 관찰력과 묘사력, 그리고 해학적 요소가 잘 어우러진 작품으로 평가받고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EA%B9%80%ED%99%8D%EB%8F%84_%EC%84%9C%EB%8B%B9.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("파적도", "김득신", "'파적도'는 '밤고양이가 쥐를 쫓다'라는 뜻으로, 김득신의 대표작 중 하나입니다. 이 작품은 어둠 속에서 고양이가 쥐를 쫓는 긴박한 순간을 포착하고 있습니다. 김득신은 최소한의 붓질로 고양이와 쥐의 움직임을 생동감 있게 표현했습니다. 특히 고양이의 날렵한 몸짓과 쥐의 필사적인 도주 모습이 대조적으로 그려져 있어, 긴장감을 고조시킵니다. 배경은 거의 생략되어 있어, 두 동물의 움직임에 모든 초점이 맞춰져 있습니다. 이 작품은 김득신의 뛰어난 관찰력과 섬세한 묘사력을 잘 보여주며, 동시에 동물의 본능적인 행동을 통해 자연의 섭리를 표현하고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EA%B9%80%EB%93%9D%EC%8B%A0_%ED%8C%8C%EC%A0%81%EB%8F%84.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("절규", "야묘도추에드바르 뭉크", "'절규'는 노르웨이 화가 에드바르 뭉크의 가장 유명한 작품으로, 현대 미술에 큰 영향을 미친 걸작입니다. 화면 중앙에는 머리를 감싸고 비명을 지르는 듯한 인물의 왜곡된 얼굴이 그려져 있습니다. 배경은 붉은 하늘과 푸른 피오르드로 이루어져 있으며, 이는 실제 오슬로 피오르드의 풍경에서 영감을 받았다고 합니다. 뭉크는 이 작품을 통해 현대 사회에서 느끼는 실존적 불안과 공포, 소외감을 표현했습니다. 인물의 왜곡된 형태와 강렬한 색채 대비는 내면의 감정을 극대화하여 보여주며, 이는 후의 표현주의 운동에 큰 영향을 미쳤습니다. '절규'는 단순히 한 개인의 고통을 넘어, 인류 보편의 불안과 공포를 상징하는 작품으로 해석되고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%97%90%EB%93%9C%EB%B0%94%EB%A5%B4+%EB%AD%89%ED%81%AC_%EC%A0%88%EA%B7%9C.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("단오 풍정", "신윤복", "'단오 풍정'은 신윤복의 대표작 중 하나로, 조선 시대 단오절의 풍경을 생동감 있게 묘사한 작품입니다. 단오는 음력 5월 5일로, 한 해 중 가장 양기가 강한 날로 여겨졌습니다. 이 그림에는 물가에서 노는 사람들과 그네를 타는 여인들의 모습이 섬세하게 묘사되어 있습니다. 특히 그네 타는 여인의 모습은 당시 여성들의 복식과 풍습을 잘 보여주고 있습니다. 신윤복은 인물들의 표정과 자세, 의복의 주름까지 섬세하게 표현하여 당시의 생활상을 생생하게 전달하고 있습니다. 또한 화면 구성과 색채 사용이 뛰어나 보는 이로 하여금 축제의 분위기를 느끼게 합니다. 이 작품은 조선 후기 풍속화의 대표작으로, 당시의 서민 문화와 여성의 삶을 엿볼 수 있는 중요한 역사적 자료로도 평가받고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%8B%A0%EC%9C%A4%EB%B3%B5_%EB%8B%A8%EC%98%A4%ED%92%8D%EC%A0%95.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("아테네 학당", "라파엘로", "'아테네 학당'은 이탈리아 르네상스의 거장 라파엘로가 바티칸 궁전의 교황 서재에 그린 대형 프레스코화입니다. 이 작품은 고대 그리스의 철학자들과 과학자들을 한 자리에 모아 놓은 상상의 장면을 묘사하고 있습니다. 중앙에는 플라톤과 아리스토텔레스가 서 있으며, 주변에는 소크라테스, 피타고라스, 디오게네스 등 다양한 학자들이 각자의 특징적인 포즈와 함께 묘사되어 있습니다. 라파엘로는 원근법과 건축적 요소를 활용하여 깊이 있는 공간감을 만들어냈으며, 각 인물의 표정과 자세를 통해 그들의 성격과 철학을 암시하고 있습니다. 이 작품은 르네상스 시대의 인문주의 정신을 잘 보여주며, 고대 그리스 철학에 대한 당시의 관심과 존경을 반영하고 있습니다. '아테네 학당'은 구도, 원근법, 인물 묘사 등 모든 면에서 르네상스 미술의 정점을 보여주는 걸작으로 평가받고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EB%9D%BC%ED%8C%8C%EC%97%98%EB%A1%9C_%EC%95%84%ED%85%8C%EB%84%A4+%ED%95%99%EB%8B%B9.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("인왕제색도", "정선", "'인왕제색도'는 조선 시대 진경산수화의 대가 정선의 대표작입니다. '인왕제색'은 '인왕산의 비 갠 풍경'이라는 뜻으로, 비가 갠 후의 인왕산 풍경을 묘사하고 있습니다. 정선은 실제 인왕산의 모습을 바탕으로 하되, 자신만의 독특한 화법으로 재해석하여 표현했습니다. 화면 중앙에는 웅장한 인왕산이 자리 잡고 있으며, 산 아래로는 안개가 걷히는 모습이 섬세하게 묘사되어 있습니다. 정선은 농묵과 담묵을 절묘하게 사용하여 습기 찬 대기와 바위의 질감을 효과적으로 표현했습니다. 또한, 점점이 찍은 듯한 준법(皴法)을 사용하여 바위의 질감을 생생하게 나타냈습니다. 이 작품은 단순히 풍경을 그린 것이 아니라, 한국적 산수의 정취와 기상을 담아낸 작품으로 평가받고 있습니다. '인왕제색도'는 조선 후기 회화의 정수를 보여주는 작품으로, 한국 미술사에서 매우 중요한 위치를 차지하고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%A0%95%EC%84%A0_%EC%9D%B8%EC%99%95%EC%A0%9C%EC%83%89%EB%8F%84.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("가시 목걸이 자화상", "프리다 칼로", "'가시 목걸이 자화상'은 멕시코의 화가 프리다 칼로의 가장 유명한 작품 중 하나입니다. 이 그림에서 칼로는 자신의 얼굴을 정면으로 그리고 있으며, 목에는 가시 목걸이를 두르고 있습니다. 가시 목걸이는 그녀의 목을 찌르고 있으며, 피를 흘리고 있습니다. 어깨에는 죽은 벌새가 매달려 있습니다. 배경에는 나뭇가지와 나비, 표범 등이 그려져 있습니다. 이 작품은 칼로의 육체적, 정신적 고통을 상징적으로 표현하고 있습니다. 가시 목걸이는 그녀가 겪은 고통을, 죽은 벌새는 희망이나 자유의 상실을, 나비는 변화와 부활을 상징한다고 해석됩니다. 칼로 특유의 강렬한 색채와 섬세한 묘사는 그녀의 내면 세계를 더욱 생생하게 전달합니다. 이 자화상은 단순한 초상화를 넘어, 칼로의 개인적 고통과 멕시코의 문화적 정체성, 여성으로서의 경험을 복합적으로 담아낸 작품입니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%ED%94%84%EB%A6%AC%EB%8B%A4+%EC%B9%BC%EB%A1%9C_%EA%B0%80%EC%8B%9C+%EB%AA%A9%EA%B1%B8%EC%9D%B4%EB%A5%BC+%ED%95%9C+%EC%9E%90%ED%99%94%EC%83%81.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("미산이곡", "장승업", "'미산이곡'은 '아름다운 산과 기이한 계곡'이라는 뜻으로, 장승업의 대표적인 산수화 작품입니다. 이 그림은 실제 존재하는 특정 장소를 그린 것이 아니라, 장승업의 상상력과 기교가 결합된 이상적인 산수의 모습을 표현한 것입니다. 화면에는 웅장한 산들과 깊은 계곡, 폭포, 안개 등이 묘사되어 있어 신비로운 분위기를 자아냅니다. 장승업은 전통적인 산수화 기법을 바탕으로 하되, 자유분방한 필치와 대담한 구도를 통해 자신만의 독특한 화풍을 만들어냈습니다. 특히 산의 윤곽을 표현한 묵선의 강약과 농담의 변화, 안개와 구름을 표현한 담묵의 사용 등에서 그의 뛰어난 기량을 엿볼 수 있습니다. '미산이곡'은 조선 말기 회화의 새로운 경향을 보여주는 작품으로, 전통과 혁신이 조화를 이룬 걸작으로 평가받고 있습니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%9E%A5%EC%8A%B9%EC%97%85_%EB%AF%B8%EC%82%B0%EC%9D%B4%EA%B3%A1.jpg");

INSERT INTO product (product_name, product_writer, description, product_url)
VALUES ("진주 귀걸이를 한 소녀", "요하네스 페르메이르", "'진주 귀걸이를 한 소녀'는 네덜란드 바로크 시대의 대가 요하네스 페르메이르의 가장 유명한 작품 중 하나입니다. 이 그림은 어두운 배경 속에서 밝게 빛나는 소녀의 얼굴을 묘사하고 있습니다. 소녀는 관람자를 향해 고개를 돌리고 있으며, 그녀의 맑은 눈동자가 인상적입니다. 제목에서 언급된 진주 귀걸이는 소녀의 왼쪽 귀에 달려 있으며, 빛을 받아 은은하게 빛나고 있습니다. 페르메이르는 빛과 그림자의 대비를 절묘하게 사용하여 소녀의 얼굴과 터번, 그리고 귀걸이의 질감을 생생하게 표현했습니다. 특히 소녀의 입술과 눈에 반사된 빛의 표현은 매우 섬세합니다. 이 작품은 단순한 구도와 제한된 색채 사용에도 불구하고 강렬한 시각적 효과를 만들어내며, 관람자의 시선을 사로잡습니다. '진주 귀걸이를 한 소녀'는 페르메이르의 뛰어난 관찰력과 빛의 처리 기술을 보여주는 대표작으로, 서양 미술사에서 가장 사랑받는 작품 중 하나로 꼽힙니다.", "https://dosunsang.s3.ap-southeast-2.amazonaws.com/product/%EC%82%AC%EC%A7%84/%EC%9A%94%ED%95%98%EB%84%A4%EC%8A%A4+%ED%8E%98%EB%A5%B4%EB%A9%94%EC%9D%B4%EB%A5%B4_%EC%A7%84%EC%A3%BC+%EA%B7%80%EA%B1%B8%EC%9D%B4%EB%A5%BC+%ED%95%9C+%EC%86%8C%EB%85%80.jpg");