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
    private static final double TICKS_PER_REVOLUTION = 537.7;
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

        // check if motors are not null


        //Sets direction of motors, this'll be different for each season and the way the mnotor is mounted
        //Make sure to use the MotorDirectionFinder class to find the directions of motors
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);




    }

    public void resetEncoders() {
        if (leftFront != null) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (rightFront != null) {
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (rightRear != null) {
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (leftRear != null) {
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (enrique != null) {
            enrique.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            enrique.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

        if (leftFront != null) {
            leftFront.setPower(0);
        }
        if (rightFront != null) {
            rightFront.setPower(0);
        }
        if (rightRear != null) {
            rightRear.setPower(0);
        }
        if (leftRear != null) {
            leftRear.setPower(0);
        }
        if (enrique != null) {
            enrique.setPower(0);
        }
    }

    public void strafe(double speed, double distance, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * INCHES_TO_TICKS);
        newRightFrontTarget = rightFront.getCurrentPosition() - (int)(distance * INCHES_TO_TICKS);
        newLeftRearTarget = leftRear.getCurrentPosition() - (int)(distance * INCHES_TO_TICKS);
        newRightRearTarget = rightRear.getCurrentPosition() + (int)(distance * INCHES_TO_TICKS);

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        double startTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startTime) < timeoutS * 1000 &&
                (leftFront.isBusy() && rightFront.isBusy()
                        && leftRear.isBusy() && rightRear.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }





        public void setEnriqueHigh() {

        if (enrique.getCurrentPosition() != 3700) {
            enrique.setPower(0.8);
            enrique.setTargetPosition(3700);
            enrique.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setEnriqueMedium()
        {
            if (enrique.getCurrentPosition() != 2600) {
                enrique.setPower(0.8);
                enrique.setTargetPosition(2600);
                enrique.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    public void setEnriqueShort() {
        if (enrique.getCurrentPosition() != 1400) {
            enrique.setPower(0.8);
            enrique.setTargetPosition(1400);
            enrique.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setEnriqueJunction()
    {
    if (enrique.getCurrentPosition() != 140) {
        enrique.setPower(0.8);
        enrique.setTargetPosition(140);
        enrique.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    }



    public void setDjkhalidOpen()
    {
        djkhalid.setPosition(0.0);
    }

    public void setDjkhalidClose()
    {
        djkhalid.setPosition(1.0);
    }










}
