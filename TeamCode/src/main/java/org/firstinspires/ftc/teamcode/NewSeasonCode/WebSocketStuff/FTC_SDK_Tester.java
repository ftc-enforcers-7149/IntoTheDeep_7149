package org.firstinspires.ftc.teamcode.NewSeasonCode.WebSocketStuff;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Field;


public class FTC_SDK_Tester {

    private static FtcEventLoop eventLoop;
    Field f;

    //@OnCreateEventLoop
    public static void getEventLoop(Context context, FtcEventLoop eventLoop) {

        FTC_SDK_Tester.eventLoop = eventLoop;

    }

    //@OnCreate
    public static void start(Context context) {
        //start stuff here (like initialization and whatnot
        OpModeManagerImpl manager = eventLoop.getOpModeManager();

        HardwareMap map = manager.getHardwareMap();  //how to get manager

    }
}
