package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robot3050 on 4/11/2018.
 */

    @TeleOp(name="3050: Teleop 2017 newer", group="3050")
//@Disabled
    public class Teleop2017newer extends OpMode
    {
        Hardware3050 robot = new Hardware3050();
        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        @Override
        public void init () {
            robot.init(hardwareMap); //Is initialized in Hardware3050
        }

        @Override
        public void loop() {
            float leftY = -gamepad1.left_stick_y;
            float leftX = -gamepad1.left_stick_x;
            float rightY = -gamepad1.right_stick_y;
            float rightX = -gamepad1.right_stick_x;

            robot.leftMotor.setPower(leftY);
            robot.leftMotor.setPower(leftX);
            robot.rightMotor.setPower(rightY);
            robot.rightMotor.setPower(rightX);

        }

    }







