package org.firstinspires.ftc.teamcode.NewSeasonCode.WebSocketStuff;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;


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

        //gets the first motor in the motor channel (not by name, but by port number!)
        DcMotorEx motor = (DcMotorEx) map.dcMotor.iterator().next();

        //automatically do this on init in special opmodes to fill out both chub and exhub objects
        //can create a master debug file that allows easy control of each actuator by just changing
        //the port number instead of the config each time
        List<DcMotorEx> motorList = new ArrayList<>();
        map.dcMotor.iterator().forEachRemaining(m -> motorList.add((DcMotorEx) m));

        //gets the name of the 2nd motor in the list (from the config)
        String name = map.getNamesOf(motorList.get(1)).iterator().next();

        DcMotorEx fl = motorList.get(0);
        DcMotorEx bl = motorList.get(1);

    }
}
