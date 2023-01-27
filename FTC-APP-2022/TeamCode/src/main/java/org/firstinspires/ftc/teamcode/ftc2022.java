package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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



    public BNO055IMU imu;



    //SENSORS:

    //This is the REV integrated gyro sensor, this GyroSensor class could be used for any gyro sensor, but this specific case is for the integrated REV one


    public Telemetry telemetry = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Declare encoder constants
    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double WHEEL_DIAMETER = 3.77953; // inches
    private static final double GEAR_RATIO = 1;


    // Declare conversion constants
    // This is for Encoder driving
    private static final double INCHES_TO_TICKS = 42.25;
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

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);



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

        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
/*
    public void strafe(double speed, double distance, double timeoutS) {
        Orientation angles;
        Acceleration gravity;

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
        while ((System.currentTimeMillis() - startTime) < timeoutS * 1000 && (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {

            // Read current IMU data and adjust motor power or position targets as needed
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

            double correction = angles.firstAngle; // Use second angle for correction
            leftFront.setPower(Math.abs(speed) - correction);
            rightFront.setPower(Math.abs(speed) + correction);
            leftRear.setPower(Math.abs(speed) + correction);
            rightRear.setPower(Math.abs(speed)-correction);
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition());
            telemetry.addData("IMU Angle", angles.firstAngle);
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
*/

    public void forward(double speed, double distance)
    {
        int targetPosition;
        targetPosition = (leftFront.getCurrentPosition())+(int)(distance*INCHES_TO_TICKS);
        leftFront.setTargetPosition(targetPosition);
        leftRear.setTargetPosition(targetPosition);
        rightFront.setTargetPosition(targetPosition);
        rightRear.setTargetPosition(targetPosition);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);



        while(leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy() )
        {

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }

    public void strafeLeft(double speed, double distance)
    {
        int targetPosition;

        targetPosition = (leftFront.getCurrentPosition()+(int)(distance * INCHES_TO_TICKS));

        leftFront.setTargetPosition(targetPosition);
        rightRear.setTargetPosition(targetPosition);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftFront.setPower(speed);
        rightRear.setPower(speed);
        while (leftFront.isBusy() && rightRear.isBusy())
        {
            leftRear.setPower(-speed);
            rightFront.setPower(-speed);
        }

        while (leftFront.isBusy() && rightRear.isBusy() && rightFront.isBusy() && leftRear.isBusy())
        {

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }







    public void moveForward(double speed, double distance, double timeoutS) {
        int newTarget;

        // Determine new target position, and pass to motor controller
        newTarget = leftFront.getCurrentPosition() + (int)(distance * INCHES_TO_TICKS);
        leftFront.setTargetPosition(newTarget);
        rightFront.setTargetPosition(newTarget);
        leftRear.setTargetPosition(newTarget);
        rightRear.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

        // wait for motors to complete movement
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() < timeoutS &&
                (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())){
            // nothing here
        }

        // stop motors and reset encoders
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }







        public void turn(double degrees) {
            // Get current angle
            Orientation angles = imu.getAngularOrientation();
            double currentAngle = angles.firstAngle;

            // Calculate target angle
            double targetAngle = currentAngle + degrees;

            // Reset encoder counts
            resetEncoders();

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

            // Loop while the robot is turning
            while(angles.firstAngle < targetAngle - 1 || angles.firstAngle > targetAngle + 1) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                double correction = (targetAngle - angles.thirdAngle) / 100;

                leftFront.setPower(0.5 + correction);
                leftRear.setPower(0.5 + correction);
                rightFront.setPower(-0.5 - correction);
                rightRear.setPower(-0.5 - correction);
            }

            // Stop motors
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
