package com.dosunsang.dosunsang_server;

import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

public class MyWebSocketHandler extends TextWebSocketHandler {
    @Override
    public void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        String payload = message.getPayload();
        System.out.println("Received message: " + payload);

        if (payload.matches("\\d+")) {
            // 숫자인 경우 기존 로직 실행
            int robotId = Integer.parseInt(payload);
            handleRobotRegistration(session, robotId);
        } else if (payload.startsWith("ack")) {
            // "ack" 메시지 처리
            handleAcknowledgement(session, payload);
        } else {
            // 기타 메시지 처리
            handleOtherMessage(session, payload);
        }
    }

    private void handleRobotRegistration(WebSocketSession session, int robotId) throws Exception {
        RobotRegistration robotRegistration = RobotRegistration.getInstance();
        if (!robotRegistration.registerRobot(robotId, session)) {
            session.sendMessage(new TextMessage(robotId + " Connected"));
        } else {
            robotRegistration.eraseSession(robotId);
            session.sendMessage(new TextMessage(robotId + " Disconnected"));
        }
    }

    private void handleAcknowledgement(WebSocketSession session, String payload) throws Exception {
        // "ack" 메시지에 대한 처리 로직
        System.out.println("Acknowledgement received: " + payload);
        session.sendMessage(new TextMessage("Acknowledgement processed: " + payload));
    }

    private void handleOtherMessage(WebSocketSession session, String payload) throws Exception {
        // 기타 메시지에 대한 처리 로직
        System.out.println("Other message received: " + payload);
        session.sendMessage(new TextMessage("Message received: " + payload));
    }
}