package org.firstinspires.ftc.teamcode.NewSeasonCode;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.ftccommon.external.OnCreateMenu;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;

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
        //start stuff here (like initialization and whatnot)
    }


}
