package com.dosunsang.dosunsang_server;

import org.springframework.web.socket.WebSocketSession;

import java.util.HashMap;

public class RobotRegistration {
    private static final RobotRegistration robotRegistration = new RobotRegistration();;
    HashMap<Integer, WebSocketSession> sessions = new HashMap<>();
    int temp = 3;
    private RobotRegistration() {

    }

    public static RobotRegistration getInstance() {
        return robotRegistration;
    }

    public boolean registerRobot(Integer id, WebSocketSession session){
        if(sessions.containsKey(id)){
            return true;
        }
        sessions.put(id,session);
        //System.out.println(sessions.size());
        //System.out.println("id : " + id + "session : " + session);
        return false;
    }

    public WebSocketSession getSession(Integer id){
        if(sessions.containsKey(id)){
            System.out.println(sessions.size());
            return sessions.get(id);
        }else {
            System.out.println(sessions.size());
            return null;
        }
    }
}
