package org.firstinspires.ftc.teamcode.NewSeasonCode.WebSocketStuff;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import java.net.InetSocketAddress;
import java.util.concurrent.ConcurrentHashMap;

public class DashLinkCore extends WebSocketServer {

    private DashLink dashLinkInstance;
    private ConcurrentHashMap<WebSocket, String> clientMap;

    public static final int PORT = 8765;

    public enum MessageTypes {
         STANDARD, ROBOT_DATA,
    }


    public DashLinkCore(DashLink instance) {
        super(new InetSocketAddress(PORT));

        dashLinkInstance = instance;
    }


    @Override
    public void onOpen(WebSocket webSocket, ClientHandshake clientHandshake) {
        webSocket.send("Welcome to DashLink");

        clientMap.put(webSocket, webSocket.getRemoteSocketAddress().getHostName());
    }

    @Override
    public void onClose(WebSocket webSocket, int i, String s, boolean b) {

    }

    @Override
    public void onMessage(WebSocket webSocket, String s) {

    }

    @Override
    public void onError(WebSocket webSocket, Exception e) {

    }

    @Override
    public void onStart() {

    }

    public void broadcastAll(String s) {

        for (WebSocket client : clientMap.keySet()) {
            client.send(s);
        }

    }

    public void startSocketStream() {




    }
}
