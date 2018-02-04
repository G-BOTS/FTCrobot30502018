package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by robot3050 on 11/22/2017.
 */

@TeleOp(name="3050: Teleop 2017", group="3050")
//@Disabled
public class Teleop2017 extends OpMode
{
    Hardware3050 robot = new Hardware3050();

    @Override
    public void init()
    {
        robot.init(hardwareMap); //Is initialized in Hardware3050
    }

    @Override
    public void loop () {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        robot.leftMotor.setPower(leftY);
        robot.rightMotor.setPower(rightY);

        if (gamepad1.left_trigger > .01) {
            robot.Elevator.setPower(.8);
        }
        else if (gamepad1.left_bumper) {
            robot.Elevator.setPower(-.8);
        }
        else
        {
            robot.Elevator.setPower(0);
        }

        if(gamepad1.right_trigger > .01) {
            robot.Intake.setPower(-.6);
        }
        else if(gamepad1.right_bumper)
        {
            robot.Intake.setPower(.3);
        }
        else
        {
            robot.Intake.setPower(0);
        }

        telemetry.addData("Elevator Power", "%f", robot.Elevator.getPower());
        telemetry.update();
    }
}