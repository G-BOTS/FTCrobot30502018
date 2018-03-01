package org.firstinspires.ftc.teamcode;

/**
 * Created by robot3050 on 2/5/2018.
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="3050: RedLeftPicto", group="3050")
//@Disabled
public class RedLeftPicto extends LinearOpMode {
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
            {24, 24, 24,},
            {-88, -88, -88,},
            {12, 12, 12,},
            {-120, -120, -120,},
            {20, 30, 38,},
            {-172, -178, -170},
            {20, 12, 8,},
            {-4, -4, -4,}};
    private Integer coLumn = 0 ;//0 for right coLumn 1 for middle colomn and 2 for left coLumn
    private Integer jewel = 1 ;

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        relicTrackables.activate();

        //while (opModeIsActive()) {

        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
        }
        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
        sleep(550);

        //switch (vuMark) {
        // case LEFT: //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.LEFT;
        //  coLumn = 2;
        // case CENTER:// RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.CENTER;
        //   coLumn = 1;
        //  case RIGHT:// RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.RIGHT;
        // coLumn = 0;
        //}
        if ( vuMark == RelicRecoveryVuMark.LEFT) {
            coLumn = 2;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT){
            coLumn = 0;
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER){
            coLumn = 1;
        }


        telemetry.addData("coLumn", "%s visible", coLumn);
        telemetry.update();
        sleep(550);



//if jewej is red 1 if jewel is blue 2

       // if(jewel == 1) {
         //   gyroturn(-10, TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
            //gyroturn(-2, -TURN_SPEED, TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
        //}
        //else if(jewel == 2){
           // gyroturn(10, -TURN_SPEED, TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
           // gyroturn(-2, TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);
        //}
        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[0][coLumn], disandTurn[0][coLumn], 5.0);  // S1: Forward 24 Inches with 5 Sec timeout shoot ball

        gyroturn(disandTurn[1][coLumn], TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[0], -turndistance[0], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[2][coLumn], disandTurn[2][coLumn], 5.0); // S3:  Forward 43.3 iNCHES

        gyroturn(disandTurn[3][coLumn], TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[4][coLumn], disandTurn[4][coLumn], 5.0);// S5: Forward 12 Inches with 4 Sec timeout

        gyroturn(disandTurn[5][coLumn], TURN_SPEED, -TURN_SPEED); //encoderDrive(TURN_SPEED, TURN_SPEED, turndistance[1], -turndistance[1], 5.0);

        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[6][coLumn], disandTurn[6][coLumn], 5.0);// S5: Forward 12 Inches with 4 Sec timeout


        Outake();
        encoderDrive(DRIVE_SPEED, DRIVE_SPEED, disandTurn[7][coLumn], disandTurn[7][coLumn], 5.0);// S6: Forward 48 inches  with 4 Sec timeout
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
            while (((Math.abs(error)) > 2.0f) && (opModeIsActive())) {
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



