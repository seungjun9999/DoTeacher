package com.dosunsang.dosunsang_server.controller;

import com.dosunsang.dosunsang_server.RobotRegistration;
import com.dosunsang.dosunsang_server.dto.ProductDto;
import com.dosunsang.dosunsang_server.dto.ResultDto;
import com.dosunsang.dosunsang_server.dto.UserDto;
import com.dosunsang.dosunsang_server.service.ChatGptService;
import com.dosunsang.dosunsang_server.service.UserService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;

import java.io.IOException;
import java.util.ArrayList;

@RestController
@Slf4j
public class RecommendController {

    @Autowired
    private ChatGptService chatGptService;
    private UserService userService;
    ArrayList<Integer> productList = new ArrayList<>();

    @PostMapping("/socket/recommend/{robotId}")
    public String recommend(@RequestBody UserDto user, @PathVariable int robotId) throws IOException {
        StringBuffer sendmsg = new StringBuffer(
                "\"{id = 1, 김정희 (추사), 난맹첩} {id = 2, 아홉번째 파도, 이반 아이바조프스키} {id = 3, 부유세계 마타베이의 걸작, 우타가와 구니요시} {id = 4, 겨울 풍경, 카밀 피사로} {id = 5, 게르니카, 파블로 피카소} {id = 6, 무제(1948), 잭슨 폴록} {id = 7, 송하음다도, 심사정} {id = 8, 시계의 연속성, 살바도르 달리} {id = 9, 밤의 카페 테라스, 빈센트 반 고흐} {id = 10, 별이 빛나는 밤, 빈센트 반 고흐} {id = 11, 서당, 김홍도} {id = 12, 야묘도추, 김득신} {id = 13, 절규, 에드바르 뭉크} {id = 14, 단오 풍정, 신윤복} {id = 15, 아테네 학당, 라파엘로} {id = 16, 영묘도 대련, 오원 장승업} {id = 17, 인왕제색도, 정선} {id = 18, 가시 목걸이 자화상, 프리다 칼로} {id = 19, 미산이곡, 오원 장승업} {id = 20, 진주 귀걸이를 한 소녀, 요하네스 페르메이르} 중에서\"");
        for (String str : user.preferences) {
            sendmsg.append(str);
            sendmsg.append(" ");
        }
        sendmsg.append("을 좋아하는 사람에게 5개의 작품의 id값만을 대사 없이 ,로 나누어서 추천해줘\";");

        // System.out.println(sendmsg);
        // 거리기반 정렬 알고리즘 필요함
        try {
            String msg = chatGptService.sendMessage(String.valueOf(sendmsg));
            // log.info(String.valueOf(msg));

            RobotRegistration robotRegistration = RobotRegistration.getInstance();
            WebSocketSession session = robotRegistration.getSession(robotId);

            if (session != null) {
                session.sendMessage(new TextMessage(msg));

            } else {
                log.info("null");
                return null;
            }

            // msg 분리
            String[] stringArray = msg.split(", ");

            for (String str : stringArray) {
                productList.add(Integer.parseInt(str));
                // log.info(str);
            }
            // productList 로봇에 전송하고 프론트로 반환하는 부분 필요함

        } catch (IOException e) {
            e.printStackTrace();
        }
        return "hi";
    }


    @PostMapping("/socket/picture/{robotId}")
    public ResultDto<String> picture(@PathVariable int robotId) throws IOException {
        RobotRegistration robotRegistration = RobotRegistration.getInstance();
        WebSocketSession session = robotRegistration.getSession(robotId);
        try {
            session.sendMessage(new TextMessage("photo"));
            return ResultDto.res(HttpStatus.OK, "성공", "photo");
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    @PostMapping("/socket/next/{robotId}")
    public ResultDto<String> next(@PathVariable int robotId) throws IOException {
        RobotRegistration robotRegistration = RobotRegistration.getInstance();
        WebSocketSession session = robotRegistration.getSession(robotId);
        try {
            session.sendMessage(new TextMessage("next"));
            return ResultDto.res(HttpStatus.OK, "성공", "next");
        } catch (Exception e) {
            return ResultDto.res(HttpStatus.BAD_REQUEST, "실패");
        }
    }

    public ArrayList<Integer> recommendPath(ArrayList<Integer> list) {
        return null;
    }
}