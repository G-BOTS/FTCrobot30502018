package org.firstinspires.ftc.robotcontroller.external.samples;

/*
 * Created by robot3050 on 1/4/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="3050: BlueLeft18", group="3050")
//@Disabled
public class BlueLeft18 extends LinearOpMode {

    Hardware3050 robot = new Hardware3050();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.3;

    private double distance[] = {12,12,12,12,12 };
    private float turndistance[] = {14, -14,-7};//positive turns to the right, negative turns to the left,  14 for 90deg
    private float disandTurn[][] = {
            {24,24,24},
            {-45,-88,-88},
            {8,11,18},
            {3,3,3},
            {4,8,8},
            {-4,-4,-4}};
    private Integer column = 1;


    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.addData("Gyro Heading:", "%.4f", getHeading());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Gyro Heading:", "%.2f", getHeading());
        telemetry.update();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //Drive(.5f,.5f,1.0f);
        //Outake();
        // Drive(-.5f,-.5f,.19f);
        //DriveTicksHeading(0.5f, 12, 0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[0][column], disandTurn[0][column], 5.0);  // S1: Forward 24 Inches with 5 Sec timeout shoot ball

        gyroturn(disandTurn[1][column], TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[0], -turndistance[0], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[2][column], disandTurn[2][column], 5.0); // S3:  Forward 43.3 iNCHES

        gyroturn(disandTurn[3][column], -TURN_SPEED, TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[column], -turndistance[column], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[4][column], disandTurn[4][column], 5.0);// S5: Forward 12 Inches with 4 Sec timeout

        Outake();

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[5][column], disandTurn[5][column], 5.0);// S6: Forward 48 inches  with 4 Sec timeout

        //gyroturn(40, -TURN_SPEED, TURN_SPEED);

        // encoderDrive(DRIVE_SPEED, DRIVE_SPEED, distance[4], distance[4], 5.0);// S8: Forward 48 inches  with 4 Sec timeout

        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        //encoderdrive(DRIVE_SPEED,DRIVE SPEED)

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void DriveTimed(float power, float time)
    {

    }

    public void DriveTicksHeading(float forward,float inches,float desheading)
    {
        double target = inches * COUNTS_PER_INCH;
        float MAINTAIN = desheading;
        float gyro_P = .6f;

        while((robot.leftMotor.getCurrentPosition() < target)&&(robot.rightMotor.getCurrentPosition() < target)&&(opModeIsActive()))
        {
            float err = MAINTAIN - getHeading();
            float turn = err * gyro_P;

            robot.leftMotor.setPower(forward + turn);
            robot.rightMotor.setPower(forward + turn);
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void encoderDrive(double leftspeed, double rightspeed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(leftspeed));
            robot.rightMotor.setPower(Math.abs(rightspeed));

            //        keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Gyro Heading:", "%.2f", getHeading());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
    public void Drive(double leftspeed, double rightspeed,double timeoutS)
    {
        runtime.reset();
        while(runtime.seconds() < timeoutS) {
            robot.leftMotor.setPower(leftspeed);
            robot.rightMotor.setPower(rightspeed);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

    }
    public void Outake()
    {
        runtime.reset();
        while(runtime.seconds()< .7) {
            robot.Intake.setPower(-1.0f);
        }
        robot.Intake.setPower(0);
    }

    public void gyroturn(float desheading, double leftspeed, double rightspeed)
    {
        float error;

        if(opModeIsActive()) {
            error = getHeading() - desheading;
            while (((Math.abs(error)) > 1.0f) && (opModeIsActive())) {
                telemetry.addData("Path1", "Aiming to %7f :%7f", error, desheading);
                telemetry.addData("Gyro Heading:", "%.2f", getHeading());

                telemetry.update();
                robot.leftMotor.setPower(leftspeed);
                robot.rightMotor.setPower(rightspeed);

                error = getHeading() - desheading;
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }

    float getHeading()
    {
        return robot.Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}

