import boto3
import mysql.connector
from urllib.parse import quote
from datetime import datetime
import os

# AWS S3 클라이언트 설정
s3 = boto3.client('s3',
    aws_access_key_id='AKIAQE43KBOTPRSUYNPE',
    aws_secret_access_key='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',
    region_name='ap-southeast-2'
)

# MySQL 데이터베이스 연결
db = mysql.connector.connect(
    host="i11d102.p.ssafy.io",
    user="root",
    password="ssafyd102wjdwo",
    database="dosunsangdb",
    port=3307
)
cursor = db.cursor()

# S3 버킷과 프리픽스 설정
bucket_name = 'dosunsang'
prefix = 'pet_picture/myphoto/'

# S3 객체 리스트 가져오기
response = s3.list_objects_v2(Bucket=bucket_name, Prefix=prefix)

# 이미지 파일 확장자 리스트
image_extensions = ['.jpg', '.jpeg', '.png', '.gif', '.bmp', '.webp']

# 각 객체에 대해 처리
for obj in response.get('Contents', []):
    # 파일 이름 추출
    file_name = obj['Key'].split('/')[-1]
    
    # 파일 확장자 확인
    _, file_extension = os.path.splitext(file_name)
    
    # 실제 파일이고 이미지 확장자를 가진 경우에만 처리
    if file_extension.lower() in image_extensions and obj['Size'] > 0:
        # S3 URL 생성
        image_url = f"https://{bucket_name}.s3.amazonaws.com/{quote(obj['Key'])}"
        
        # 현재 시간 가져오기
        created_at = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # DB에 삽입
        sql = "INSERT INTO photo (file_name, image_url, user_id, created_at) VALUES (%s, %s, %s, %s)"
        val = (file_name, image_url, 1, created_at)
        cursor.execute(sql, val)
        
        # 방금 삽입된 photo의 id 가져오기
        photo_id = cursor.lastrowid
        
        # description 업데이트
        update_sql = "UPDATE photo SET description = %s WHERE photo_id = %s"
        update_val = (f"Photo ID: {photo_id}", photo_id)
        cursor.execute(update_sql, update_val)
        
        print(f"Inserted and updated: {file_name} with Photo ID: {photo_id}")
    else:
        print(f"Skipped: {obj['Key']} (not an image file or empty)")

# 변경사항 커밋
db.commit()

print(f"{cursor.rowcount} records processed.")

# 연결 종료
cursor.close()
db.close()