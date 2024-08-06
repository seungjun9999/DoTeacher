package com.dosunsang.dosunsang_server.controller;

import com.dosunsang.dosunsang_server.service.ChatGptService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;

@RestController
@Slf4j
@RequestMapping("/gpt")
public class ChatGptController {

    @Autowired
    private ChatGptService chatGptService;
    @PostMapping("/send")
    public String sendMessage(@RequestBody String message) {
        try {
            return chatGptService.sendMessage(message);
        } catch (IOException e) {
            return "Error: " + e.getMessage();
        }
    }
}
