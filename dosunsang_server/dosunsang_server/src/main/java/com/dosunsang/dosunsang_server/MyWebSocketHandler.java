package com.dosunsang.dosunsang_server;

import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;


public class MyWebSocketHandler extends TextWebSocketHandler {
    @Override
    public void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        String payload = message.getPayload();
        System.out.println(Integer.parseInt(payload));
        int robotId = Integer.parseInt(payload);

        RobotRegistration robotRegistration = RobotRegistration.getInstance();
        if(!robotRegistration.registerRobot(robotId, session)){
            session.sendMessage(new TextMessage(payload + " Connected"));
        }else{
            robotRegistration.eraseSession(robotId);
            session.sendMessage(new TextMessage(payload + " Disconnected"));
        }
    }
}
