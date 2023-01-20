package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.android.dex.Code;

import java.util.Arrays;
import java.util.Collections;

public class ftc2022 {

    private double StrafeSpeed = 50;
    private int TurnSpeed = 50;


    private boolean runPosition = false;

    LynxModule driveHub = null;

    //MOTORS:
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftRear = null;
    public DcMotor enrique = null;
    public CRServo djkhalid = null;


    //SERVOS:
    public Telemetry telemetry = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Declare encoder constants
    private static final double TICKS_PER_REVOLUTION = 1120;
    private static final double WHEEL_DIAMETER = 4; // inches
    private static final double GEAR_RATIO = 1;


    // Declare conversion constants
    private static final double INCHES_TO_TICKS = TICKS_PER_REVOLUTION / (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
    private static final double DEGREES_TO_TICKS = INCHES_TO_TICKS / (WHEEL_DIAMETER * Math.PI / 180);

    // Declare PID constants
    private static final double KP = 0.01;
    private static final double KI = 0.001;
    private static final double KD = 0.1;

    // Declare timeout variable
    private ElapsedTime timeout = new ElapsedTime();

    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telem;

        // Define REV Expansion hubs
        driveHub = hwMap.get(LynxModule.class, "Expansion Hub 2");

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "LF");
        rightFront = hwMap.get(DcMotor.class, "RF");
        rightRear = hwMap.get(DcMotor.class, "RR");
        leftRear = hwMap.get(DcMotor.class, "LR");
        enrique = hwMap.get(DcMotor.class, "EN");
        djkhalid = hwMap.get(CRServo.class, "DJ");



        //armVertical = hwMap.get(DcMotor.class, "armVertical");
        //armHorizontal = hwMap.get(DcMotor.class, "armHorizontal");
        //counter = hwMap.get(DcMotor.class, "counter");


        //Define servos
        //grabber1 = hwMap.get(Servo.class, "grab1");
        //grabber2 = hwMap.get(Servo.class, "grab2");

        // Set all motors to zero power
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        enrique.setPower(0);


        //armVertical.setPower(0);
        //armHorizontal.setPower(0);
        //counter.setPower(0);
        //arm1.setPower(0);
        //arm2.setPower(0);


        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //runToPosition(false);

        // Define and initialize ALL installed servos.
        //intakeMechanismLeft = hwMap.get(Servo.class, "intakeMechanismLeft");
        //intakeMechanismRight = hwMap.get(Servo.class, "intakeMechanismRight");


//        setCapArm(1);

        //setIntakeMechanism(false);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void setDrivePower(double fl, double bl, double fr, double br) {
        if (runPosition) {
            rightRear.setPower(br);
            rightFront.setPower(fr);
            leftFront.setPower(fl);
            leftRear.setPower(bl);


        } else {
            rightRear.setPower(br);
            rightFront.setPower(fr);
            leftFront.setPower(fl);
            leftRear.setPower(bl);
        }
    }

    public void strafe(double xDistance, double yDistance, double speed, long timeout) {
        // Reset encoders and timeout
        resetEncoders();
        this.timeout.reset();

        // Calculate target tick counts
        int xTarget = (int) (xDistance * INCHES_TO_TICKS);
        int yTarget = (int) (yDistance * INCHES_TO_TICKS);

        // Set target tick counts
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + xTarget + yTarget);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - xTarget + yTarget);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - xTarget + yTarget);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + xTarget + yTarget);

        // Enable PID and set speed
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);

        // Wait for motors to reach target position or timeout


        // Update telemetry
        telemetry.addData("X Target", xTarget);
        telemetry.addData("Y Target", yTarget);
        telemetry.addData("Front Left Position", leftFront.getCurrentPosition());
        telemetry.addData("Front Right Position", rightFront.getCurrentPosition());
        telemetry.addData("Back Left Position", leftRear.getCurrentPosition());
        telemetry.addData("Back Right Position", rightRear.getCurrentPosition());
        telemetry.update();


        // Stop and reset motors

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }






    //Time based method for strafing forward, NO ENCODERS

    public void strafeforward(double distance)
    {
        int time = (int)(distance/ StrafeSpeed*1000);

        leftFront.setPower(StrafeSpeed);
        leftRear.setPower(StrafeSpeed);
        rightFront.setPower(StrafeSpeed);
        rightRear.setPower(StrafeSpeed);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void turnRight(double degrees)
    {
        leftFront.setPower(TurnSpeed);
        leftRear.setPower(TurnSpeed);
        rightFront.setPower(TurnSpeed);
        rightRear.setPower(TurnSpeed);

        int time = (int)(degrees/TurnSpeed * 1000);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // stop the motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }

    public void turnLeft(double degrees)
    {
        leftFront.setPower(-TurnSpeed);
        leftRear.setPower(-TurnSpeed);
        rightFront.setPower(-TurnSpeed);
        rightRear.setPower(-TurnSpeed);

        int time = (int)(degrees/TurnSpeed * 1000);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // stop the motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }




    public void turn(double degrees) {
        // Reset encoder counts
        resetEncoders();

        // Calculate target encoder counts
        int target = (int) (DEGREES_TO_TICKS * degrees);

        // Set target position for each motor
        leftFront.setTargetPosition(target);
        leftRear.setTargetPosition(target);
        rightFront.setTargetPosition(-target);
        rightRear.setTargetPosition(-target);

        // Set motor power
        leftFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(-0.5);

        // Use encoders to turn
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Stop motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void setEnrique(double power)
    { enrique.setPower(power);}

    public void setDjkhalid(double x)
    {
        djkhalid.setPower(x);
    }






}
