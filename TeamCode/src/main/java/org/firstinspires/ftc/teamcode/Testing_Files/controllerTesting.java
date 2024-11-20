package org.firstinspires.ftc.teamcode.Testing_Files;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@TeleOp(name="Controller Tester")
public class controllerTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Gamepad Right Joystick x:", gamepad1.right_stick_x);
            telemetry.addData("Gamepad Left Joystick x:", gamepad1.left_stick_x);
            telemetry.addData("Gamepad Right Joystick y:", gamepad1.right_stick_y);
            telemetry.addData("Gamepad Left Joystick y:", gamepad1.left_stick_y);
            telemetry.addData("Gamepad triangle:", gamepad1.triangle);
            telemetry.addData("Gamepad cross:", gamepad1.cross);
            telemetry.addData("Gamepad circle:", gamepad1.circle);
            telemetry.addData("Gamepad square:", gamepad1.square);
            telemetry.addData("Gamepad right trigger:", gamepad1.right_trigger);
            telemetry.addData("Gamepad left trigger:", gamepad1.left_trigger);
            telemetry.addData("Gamepad right bumper:", gamepad1.right_bumper);
            telemetry.addData("Gamepad left bumper:", gamepad1.left_bumper);
            telemetry.addData("Gamepad dpad up:", gamepad1.dpad_up);
            telemetry.addData("Gamepad dpad down:", gamepad1.dpad_down);
            telemetry.addData("Gamepad dpad left:", gamepad1.dpad_left);
            telemetry.addData("Gamepad dpad right:", gamepad1.dpad_right);
            telemetry.addData("Gamepad share:", gamepad1.share);
            telemetry.addData("Gamepad options:", gamepad1.options);

            telemetry.update();

        }


    }

}
