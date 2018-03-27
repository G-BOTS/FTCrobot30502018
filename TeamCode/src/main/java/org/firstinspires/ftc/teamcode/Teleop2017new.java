package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robot3050 on 3/7/2018.
 */

@TeleOp(name="3050: Teleop 2017 new", group="3050")
//@Disabled
public class Teleop2017new extends OpMode
{
    Hardware3050 robot = new Hardware3050();
    // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    @Override
    public void init () {

        robot.init(hardwareMap); //Is initialized in Hardware3050

        robot.JewelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.JewelArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        robot.leftMotor.setPower(leftY);
        robot.rightMotor.setPower(rightY);

        //if (gamepad1.left_trigger > .01&&digitalTouch.getState()!=false) {
        if (gamepad1.left_trigger > .01) {
            robot.Elevator.setPower(.8);
        } else if (gamepad1.left_bumper) {
            robot.Elevator.setPower(-.8);
        } else {
            robot.Elevator.setPower(0);
        }

        if (gamepad1.right_trigger > .01) {
            robot.JewelArm.setPower(.1);
        } else if (gamepad1.right_bumper) {
            robot.JewelArm.setPower(-.3);
        } else {
            robot.JewelArm.setPower(0);
        }
        if (gamepad1.a) {
            robot.Intake.setPower(.6);
        }
         else if (gamepad1.b) {
            robot.Intake.setPower(-.6);
        }
         else {
            robot.Intake.setPower(0);
        }
        telemetry.addData("Elevator Power", "%f", robot.Elevator.getPower());
        telemetry.update();
    }

}



