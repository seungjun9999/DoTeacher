FROM gradle:8.5-jdk21 as builder
WORKDIR /build

# 그래들 파일이 변경되었을 때만 새롭게 의존패키지 다운로드 받게함.
COPY build.gradle settings.gradle /build/
RUN gradle build -x test --parallel --continue > /dev/null 2>&1 || true

# 빌더 이미지에서 애플리케이션 빌드
COPY . /build
RUN gradle build -x test --parallel

# APP
FROM eclipse-temurin:21-jre-jammy
WORKDIR /app

# 환경 변수 설정
ENV TEMP_UPLOAD_DIR=/tmp/upload

# 임시 업로드 디렉토리 생성 및 권한 설정
RUN mkdir -p ${TEMP_UPLOAD_DIR} && chmod 777 ${TEMP_UPLOAD_DIR}

# 빌더 이미지에서 jar 파일만 복사
COPY --from=builder /build/build/libs/*.jar app.jar

EXPOSE 8080

# root 대신 nobody 권한으로 실행
USER nobody
ENTRYPOINT [ \
   "java", \
   "-jar", \
   "-Djava.security.egd=file:/dev/./urandom", \
   "-Dsun.net.inetaddr.ttl=0", \
   "app.jar" \
]