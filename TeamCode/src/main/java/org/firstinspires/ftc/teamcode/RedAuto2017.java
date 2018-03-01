
package org.firstinspires.ftc.teamcode; /**
 * Created by ferasmus on 2/3/2017.
 */
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class RedAuto2017  extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private double distance[] = {24,43.3f,12,48,48 };
    private float gyrodegree[] = {56.3f, -33.7f,-5.0f};

public class RedAuto2017 {
}
//WaitForStart*/

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class RedAuto2017 extends OpMode
{
    Hardware3050 robot = new Hardware3050();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //a list of all the system states
    private enum State {
        STATE_INITIAL,
        STATE_DRIVE_TO_SHOOT,
        STATE_LOAD_SHOOTER,
        STATE_SHOOT_SHOOTER,
        STATE_RELOAD_NEXT_SHOT,
        STATE_DRIVE_TO_BEACON,
        STATE_STOP,
    }
//pathseg are defined as distances for left wheel and right wheel and then the power to the motors
    final PathSeg[] mshootpospath = {
            new PathSeg( 12, 12, 0.8), //forward
            new PathSeg( 0.0, 6.0, 0.6), //slowdown and rotate  left
    };

    final PathSeg[] mParalleltowallpath = {
            new PathSeg( 3.2, 3.2, 0.6), //forward
            new PathSeg( -4.0, 4.0, 0.4), //slowdown and rotate  left
    };

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.8;     // For figuring circumference
    static final double TicksPerInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    final double TriggerForShoot=500;
    final double ShooterFullExtend=500;

    //robot devices
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor Elevator;
    public DcMotor Trigger;
    public DcMotor Lift;
    public DcMotor Intake;
    public Servo rightClaw;
    public GyroSensor Gyro;

    private int mLeftEncoder;
    private int mRightEncoder;
    //
    //loop cycle time stats variables
    public ElapsedTime mRunTime = new ElapsedTime();  //time into round

    private ElapsedTime mStateTime = new ElapsedTime();  //time into state

    private State  mcurrentState; //statemachine state
    private PathSeg[] mcurrentPath; //array to hold path
    private int     mcurrentSeg; //Index of the current seg in the current path

    //init
    @Override
    public void init()
    {
        //initialise class members

        /*leftMotor   = hardwareMap.dcMotor.get("left_drive");
        rightMotor  = hardwareMap.dcMotor.get("right_drive");
        Elevator = hardwareMap.dcMotor.get("Elevator");
        Trigger = hardwareMap.dcMotor.get("Trigger");
        Lift = hardwareMap.dcMotor.get("Lift");
        Intake = hardwareMap.dcMotor.get("Intake");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Elevator.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Trigger.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);

        Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Trigger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Gyro = hardwareMap.gyroSensor.get("Gyro");*/

        setDrivePower(0,0); //make sure the robot is standing still
        resetDriveEncoders();
    }

    //loop
    //@Override
    public void init_loop() {
        resetDriveEncoders();
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
    }

    @Override
    public void start()
    {
        setDriveSpeed(0, 0);
        runToPosition();
        mRunTime.reset();
        newState(State.STATE_INITIAL);
    }
    @Override
    public void loop()
    {

        //telemetry.addData("0", String.format("%4.1f", mStateTime.time()) + mcurrentState.toString());

        switch (mcurrentState)
        {
            case STATE_INITIAL:// stay in this state until both encoders are zero
                telemetry.addLine("State_Initial");
                if (encodersAtZero())
                {
                    startPath(mshootpospath);  // Load path to shooting position
                    Lift.setPower(-.1);
                    newState(State.STATE_DRIVE_TO_SHOOT);
                }
                else
                {
                    // display diagnostic data for this sate{}
                    //telemetry.addData("1", String.format("L %5d - R %5d", getLeftPosition(), getRightPosition()));
                }

                break;

            case STATE_DRIVE_TO_SHOOT:// stay in this state until both encoders are zero
                telemetry.addLine("State_Drive_to_Shoot");
                if (pathComplete())
                {
                    useConstantPower();
                    setDrivePower(0, 0);
                    Elevator.setTargetPosition(1900);
                    Elevator.setPower(0.75);
                    newState(State.STATE_LOAD_SHOOTER);
                }
                else
                {
                    // display diagnostic data for this sate{}
                    //telemetry.addData("1", String.format("L:R %7f:%7f", getLeftPosition(), getRightPosition()));
                }
                break;

            case STATE_LOAD_SHOOTER:
                telemetry.addLine("State_Load_Shooter");
                if(getElevatorPosition() > 1700)
                {
                    Trigger.setTargetPosition(-125);
                    Trigger.setPower(0.5);
                    newState(State.STATE_SHOOT_SHOOTER);
                }
                else
                {
                    //telemetry.addData("Elevator Encoder", "%.2f", getElevatorPosition());
                }
                break;

            case STATE_SHOOT_SHOOTER:
                telemetry.addLine("State_Shoot_Shooter");
                if(getTriggerPosition() < -75)
                {
                    startPath(mParalleltowallpath);
                    newState(State.STATE_DRIVE_TO_BEACON);
                }
                else
                {
                    //telemetry.addLine("Not Shot Yet");
                }
                break;

            case STATE_DRIVE_TO_BEACON:
                if(pathComplete())
                {
                    Elevator.setTargetPosition(0);
                    Elevator.setPower(-.5);
                    Trigger.setPower(0);
                    newState(State.STATE_STOP);
                }
                else
                {
                    //telemetry.addData("Drive Encoder: ", "L%.2f: R%.2f", getLeftPosition(), getRightPosition());
                }
                break;

            case STATE_STOP:
                telemetry.addLine("State_Stop");
                break;
        }
    }

    @Override
    public void stop()
    {
     //ensure that the motors are turned off
        useConstantPower();
        setDrivePower(0, 0);
    }
    //user defined utility functions here

    //transition to a new state
    private void newState(State newState)
    {//reset state time
        mStateTime.reset();
        mcurrentState = newState;
    }
    void setEncoderTarget( int LeftEncoder, int RightEncoder)
    {
        leftMotor.setTargetPosition(mLeftEncoder = LeftEncoder);
        rightMotor.setTargetPosition(mRightEncoder = RightEncoder);
    }

    void addEncoderTarget( int LeftEncoder, int RightEncoder)
    {
        leftMotor.setTargetPosition(mLeftEncoder  += LeftEncoder);
        rightMotor.setTargetPosition(mRightEncoder += RightEncoder);

    }

    void setDrivePower(double leftPower, double rightPower)
    {
      leftMotor.setPower(Range.clip(leftPower, -1,1));
        rightMotor.setPower(Range.clip(rightPower,-1,1));
    }

    void setDriveSpeed(double leftSpeed, double rightSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed);
    }

    public void runToPosition()
    {
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public  void useConstantSpeed()
    {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public  void useConstantPower()
    {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void synchEncoders()
    {
        mLeftEncoder = leftMotor.getCurrentPosition();
        mRightEncoder = rightMotor.getCurrentPosition();
    }

    public void setDriveMode(DcMotor.RunMode mode)
    {
        if (leftMotor.getMode() != mode)
        {
            leftMotor.setMode(mode);
        }

        if (rightMotor.getMode() != mode)
        {
            rightMotor.setMode(mode);
        }
    }
    int getLeftPosition()
    {
        return leftMotor.getCurrentPosition();
    }

    int getRightPosition()
    {
        return rightMotor.getCurrentPosition();
    }

    int getElevatorPosition() { return Elevator.getCurrentPosition(); }

    int getTriggerPosition() { return Trigger.getCurrentPosition(); }

    boolean moveComplete()
    {
        return ((Math.abs(getLeftPosition() - mLeftEncoder)<10)&&(Math.abs(getRightPosition() - mRightEncoder)<10));
    }

    boolean encodersAtZero()
    {
      return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }

    private void startPath(PathSeg[] path)
    {
        mcurrentPath = path; //initial path array
        mcurrentSeg=0;
        synchEncoders();  //lock in the current possition
        runToPosition();
        startSeg();  //Execute the current (firstLeg)
    }

    private void startSeg()
    {
        int Left;
        int Right;

        if(mcurrentPath !=null)
            //load up the next motion based on the current segment
        {
            Left = (int)(mcurrentPath[mcurrentSeg].mLeft*TicksPerInch);
            Right = (int)(mcurrentPath[mcurrentSeg].mRight*TicksPerInch);
            addEncoderTarget(Left, Right);
            setDriveSpeed(mcurrentPath[mcurrentSeg].mSpeed, mcurrentPath[mcurrentSeg].mSpeed);

            mcurrentSeg++;
        }
    }

    private boolean pathComplete()
    {
        if(moveComplete())
        {
            if(mcurrentSeg < mcurrentPath.length)
            {
                startSeg();
            }
            else
            {
                mcurrentPath = null;
                mcurrentSeg = 0;
                setDriveSpeed(0, 0);
                useConstantSpeed();
                return true;
            }
        }
        return false;
    }
}

class PathSeg
{
    public double mLeft;
    public double mRight;
    public double mSpeed;

    // Constructor
    public PathSeg(double Left, double Right, double Speed)
    {
        mLeft = Left;
        mRight = Right;
        mSpeed = Speed;
    }
}
