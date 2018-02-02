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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="3050: RedRightPicto", group="3050")
//@Disabled
public class RedRightPicto extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.3;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    Hardware3050 robot = new Hardware3050();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    //private double distance[] = {12, 12, 12, 12, 12};
    //private float turndistance[] = {14, -14, -7};//positive turns to the right, negative turns to the left,  14 for 90deg
    private float disandTurn[][] = {
            {24, 24, 24},
            {-88, -88, -88},
            {12, 12, 12},
            {-135, -135, -135},
            {12, 8, 4},
            {-88, -88, -88},
            {16, 20, 24},
            {-4, -4, -4}};
    private Integer column = 0 ;//0 for right column 1 for middle colomn and 2 for left column


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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Adiq0Gb/////AAAAme76+E2WhUFamptVVqcYOs8rfAWw8b48caeMVM89dEw04s+/mRV9TqcNvLkSArWax6t5dAy9ISStJNcnGfxwxfoHQIRwFTqw9i8eNoRrlu+8X2oPIAh5RKOZZnGNM6zNOveXjb2bu8yJTQ1cMCdiydnQ/Vh1mSlku+cAsNlmfcL0b69Mt2K4AsBiBppIesOQ3JDcS3g60JeaW9p+VepTG1pLPazmeBTBBGVx471G7sYfkTO0c/W6hyw61qmR+y7GJwn/ECMmXZhhHkNJCmJQy3tgAeJMdKHp62RJqYg5ZLW0FsIh7cOPRkNjpC0GmMCMn8AbtfadVZDwn+MPiF02ZbthQN1N+NEUtURP0BWB1CmA";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //telemetry.addData(">", "Press Play to start");
        //telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                switch (vuMark) {
                    case LEFT: //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.LEFT;
                        column = 2;
                    case CENTER:// RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.CENTER;
                        column = 1;
                    case RIGHT:// RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.RIGHT;
                        column = 0;

                }
                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. *
                *OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
               * telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;*/

            } else {
                telemetry.update();

            }
            //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            //telemetry.update();// this is where the program stops and loops until the 30 seconds is up

        //}


//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";


        //telemetry.addData("Gyro Heading:", "%.2f", getHeading());
        //telemetry.update();


        gyroturn(-10, TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
        gyroturn(0, -TURN_SPEED, TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
        gyroturn(10, -TURN_SPEED, TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
        gyroturn(0, TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[0][column], disandTurn[0][column], 5.0);  // S1: Forward 24 Inches with 5 Sec timeout shoot ball

        gyroturn(disandTurn[1][column], TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[0], -turndistance[0], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[2][column], disandTurn[2][column], 5.0); // S3:  Forward 43.3 iNCHES

        gyroturn(disandTurn[3][column], TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[4][column], disandTurn[4][column], 5.0);// S5: Forward 12 Inches with 4 Sec timeout
        gyroturn(disandTurn[5][column], -TURN_SPEED, TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[6][column], disandTurn[6][column], 5.0);// S5: Forward 12 Inches with 4 Sec timeout


        Outake();
        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[7][column], disandTurn[7][column], 5.0);// S6: Forward 48 inches  with 4 Sec timeout
    }
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



