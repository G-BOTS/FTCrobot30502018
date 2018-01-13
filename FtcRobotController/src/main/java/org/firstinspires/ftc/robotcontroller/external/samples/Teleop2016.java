package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
/*
 * Created by Administrator on 10/15/2016.
 */

public class Teleop2016 extends OpMode{

    DcMotor leftmotor;
    DcMotor rightmotor;

    HardwareMap hwMap = null;

    @Override
    public void init(){

        leftmotor  =hwMap.dcMotor.get("left_drive");
        rightmotor =hwMap.dcMotor.get("right_drive");

        //reverse the right motor
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
     public void loop (){
        float leftY = -gamepad1.left_stick_y;

        float rightY = -gamepad1.right_stick_y;

        leftmotor.setPower(leftY);
        rightmotor.setPower(rightY);
    }
}
