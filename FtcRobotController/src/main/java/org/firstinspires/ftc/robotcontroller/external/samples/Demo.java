package org.firstinspires.ftc.robotcontroller.external.samples;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;

/**
 * Created by ferasmus on 4/24/2017.
 */

@TeleOp (name="Demo", group="3050")
//@Disabled

     public class Demo extends OpMode {
    DcMotor leftmotor;
    DcMotor rightmotor;

    @Override
    public void init() {
        leftmotor = hardwareMap.dcMotor.get("left_drive");
        rightmotor = hardwareMap.dcMotor.get("right_drive");
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        resetStartTime();
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;

        float rightY = -gamepad1.right_stick_y;

        if (getRuntime() < 30) {
            leftmotor.setPower(leftY);
            rightmotor.setPower(rightY);
        } else if (getRuntime() > 30) {
            leftmotor.setPower(0);
            rightmotor.setPower(0);

            telemetry.addLine("You are finished! Hold A and B to restart");
            telemetry.update();

            if ((gamepad1.a) && (gamepad1.b)) {
                resetStartTime();
                ;
            }
            telemetry.addData("Runtime: ", "if", getRuntime());
            telemetry.update();
        }
    }
}