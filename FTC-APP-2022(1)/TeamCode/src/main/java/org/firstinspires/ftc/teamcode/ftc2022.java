package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ftc2022 {


    private boolean runPosition = false;

    LynxModule driveHub = null;

    //MOTORS:
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftRear = null;
    public DcMotor enrique = null;


    //SERVOS:

    //Normal Servo classes essentially rotate the servo until it reaches a specific position you put it
    //between 0 to 1, with 0.5 being the neutral position


    public Servo djkhalid = null;

    //SENSORS:

    //This is the REV integrated gyro sensor, this GyroSensor class could be used for any gyro sensor, but this specific case is for the integrated REV one


    public Telemetry telemetry = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Declare encoder constants
    private static final double TICKS_PER_REVOLUTION = 1120;
    private static final double WHEEL_DIAMETER = 4; // inches
    private static final double GEAR_RATIO = 1;


    // Declare conversion constants
    // This is for Encoder driving
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
        djkhalid = hwMap.get(Servo.class, "DJ");







        // Set all motors to zero power
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        enrique.setPower(0);

        //Sets direction of motors, this'll be different for each season and the way the mnotor is mounted
        //Make sure to use the MotorDirectionFinder class to find the directions of motors
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);




    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //This is for normal driving with teleop, to move all motors at once
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
    public void strafeForward(double inches) {
        double power = 0.3;
        double time = inches / power;
        double startTime = System.currentTimeMillis();
        double endTime = startTime + time;
        leftFront.setPower(power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(power);
        while (System.currentTimeMillis() < endTime) {
            // update the motors here
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }


/*
    void turnwithGyro(double targetAngle) {
        roblox.calibrate();
        double currentAngle = roblox.getHeading();
        double error = targetAngle - currentAngle;
        double kp = 0.03;

        while(Math.abs(error) > 2) {
            currentAngle = roblox.getHeading();
            error = targetAngle - currentAngle;
            double power = error * kp;

            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(power);
            rightRear.setPower(power);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
/*
    public void turnwithGyroPID(double targetAngle) {
        roblox.calibrate();
        double currentAngle = roblox.getHeading();
        double error = targetAngle - currentAngle;
        double kp = 0.03;
        double ki = 0.01;
        double kd = 0.02;
        double integral = 0;
        double derivative = 0;
        double previousError = 0;
        double dt = 0.001;
        while(Math.abs(error) > 2) {
            currentAngle = roblox.getHeading();
            error = targetAngle - currentAngle;
            integral += error * dt;
            derivative = (error - previousError) / dt;
            double power = kp * error + ki * integral + kd * derivative;
            previousError = error;
            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(power);
            rightRear.setPower(power);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
*/




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

    public void setDjkhalid(boolean deploy)
    {
        djkhalid.setPosition(deploy ? 0.9:0.0);

    }









}
