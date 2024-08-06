import boto3
import mysql.connector
from urllib.parse import quote

# AWS S3 클라이언트 설정
s3 = boto3.client('s3',
    aws_access_key_id='AKIAQE43KBOTPRSUYNPE',
    aws_secret_access_key='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',
    region_name='ap-southeast-2'
)

# MySQL 데이터베이스 연결
db = mysql.connector.connect(
    host="127.0.0.1",
    user="root",
    password="ssafy",
    database="dosunsangdb",
    port=3306
)
cursor = db.cursor()

# S3 버킷과 프리픽스 설정
bucket_name = 'dosunsang'
prefix = 'pet_picture/미술작품/'  # s3:// 부분 제거

# S3 객체 리스트 가져오기
response = s3.list_objects_v2(Bucket=bucket_name, Prefix=prefix)

# 각 객체에 대해 처리
for obj in response.get('Contents', []):
    # 파일 이름 추출
    file_name = obj['Key'].split('/')[-1]
    
    # '*'를 기준으로 파일 이름 분리
    parts = file_name.split('_')
    
    if len(parts) >= 3:
        product_name = parts[1].strip()
        description = parts[2].strip()
    else:
        # '*'로 구분된 부분이 충분하지 않을 경우 기본값 사용
        product_name = file_name
        description = "No description available"
    
    # S3 URL 생성
    url = f"https://{bucket_name}.s3.amazonaws.com/{quote(obj['Key'])}"
    
    # DB에 삽입
    sql = "INSERT INTO product (product_name, description, product_url) VALUES (%s, %s, %s)"
    val = (product_name, description, url)
    cursor.execute(sql, val)

# 변경사항 커밋
db.commit()

print(f"{cursor.rowcount} records inserted.")

# 연결 종료
cursor.close()
db.close()