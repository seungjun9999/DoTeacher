package com.dosunsang.dosunsang_server.service;
//gpt-4o-mini

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import okhttp3.*;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.*;

@Service
public class ChatGptService {

    private static final String OPENAI_API_URL = "https://api.openai.com/v1/chat/completions";
    private final OkHttpClient httpClient = new OkHttpClient();

    @Value("${openai.api.key}")
    private String apiKey;

    public String sendMessage(String message) throws IOException {
        // 요청 본문을 생성합니다.
        Map<String, Object> requestBodyMap = new HashMap<>();
        requestBodyMap.put("model", "gpt-4o-mini");

        List<Map<String, String>> messages = new ArrayList<>();
        Map<String, String> messageContent = new HashMap<>();
        messageContent.put("role", "user");
        messageContent.put("content", message);
        messages.add(messageContent);

        requestBodyMap.put("messages", messages);

        ObjectMapper objectMapper = new ObjectMapper();
        String jsonRequestBody = objectMapper.writeValueAsString(requestBodyMap);

        // 요청 본문을 출력합니다.
        System.out.println("Request Body: " + jsonRequestBody);

        RequestBody body = RequestBody.create(
                jsonRequestBody,
                MediaType.parse("application/json")
        );

        Request request = new Request.Builder()
                .url(OPENAI_API_URL)
                .post(body)
                .addHeader("Authorization", "Bearer " + apiKey)
                .addHeader("Content-Type", "application/json")
                .build();

        // 응답을 처리합니다.
        try (Response response = httpClient.newCall(request).execute()) {
            // 응답 코드를 출력합니다.
            System.out.println("Response Code: " + response.code());

            if (!response.isSuccessful()) {
                // 응답 본문을 UTF-8로 디코딩하여 출력합니다.
                String responseBody = new String(response.body().bytes(), StandardCharsets.UTF_8);
                System.out.println("Response Body: " + responseBody);
                throw new IOException("Unexpected code " + response);
            }

            // 성공적인 응답의 경우 본문을 UTF-8로 디코딩하여 반환합니다.
            String responseBody = new String(response.body().bytes(), StandardCharsets.UTF_8);

            // JSON 파싱
            JsonNode jsonNode = objectMapper.readTree(responseBody);

            // 예를 들어, "choices" 배열의 첫 번째 항목의 "message"의 "content" 항목을 추출합니다.
            String content = jsonNode
                    .path("choices")
                    .get(0)
                    .path("message")
                    .path("content")
                    .asText();

            return content;
        }
    }
}
