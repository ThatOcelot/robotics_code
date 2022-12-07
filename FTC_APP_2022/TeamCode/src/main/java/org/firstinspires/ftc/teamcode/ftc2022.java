package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;

public class ftc2022 {

    private boolean runPosition = false;

    LynxModule driveHub = null;

    //MOTORS:
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor carousel = null;
    public DcMotor regula1 = null;
    public DcMotor baller =  null;

    //SERVOS:
    public Telemetry telemetry = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    private static final double TICK_FACTOR = (35.0 / 45.0) * (1120 / Math.PI / 4);


    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telem;

        // Define REV Expansion hubs
        driveHub = hwMap.get(LynxModule.class, "Expansion Hub 2");

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        carousel = hwMap.get(DcMotor.class, "carousel");
        regula1 = hwMap.get(DcMotor.class, "regula");
        baller = hwMap.get(DcMotor.class, "ball");
        //armVertical = hwMap.get(DcMotor.class, "armVertical");
        //armHorizontal = hwMap.get(DcMotor.class, "armHorizontal");
        //counter = hwMap.get(DcMotor.class, "counter");


        //Define servos
        //grabber1 = hwMap.get(Servo.class, "grab1");
        //grabber2 = hwMap.get(Servo.class, "grab2");

        // Set all motors to zero power
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        carousel.setPower(0);
        regula1.setPower(0);
        baller.setPower(0);
        //armVertical.setPower(0);
        //armHorizontal.setPower(0);
        //counter.setPower(0);
        //arm1.setPower(0);
        //arm2.setPower(0);


        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //runToPosition(false);

        // Define and initialize ALL installed servos.
        //intakeMechanismLeft = hwMap.get(Servo.class, "intakeMechanismLeft");
        //intakeMechanismRight = hwMap.get(Servo.class, "intakeMechanismRight");


//        setCapArm(1);

        //setIntakeMechanism(false);
    }


    public void setDrivePower(double fl, double bl, double fr, double br) {
        if (runPosition) {
            backRight.setPower(br);
            frontRight.setPower(fr);
            frontLeft.setPower(fl);
            backLeft.setPower(bl);
        } else {
            backRight.setPower(br);
            frontRight.setPower(fr);
            frontLeft.setPower(fl);
            backLeft.setPower(bl);
        }
    }

    private int move(int fl_delta, int fr_delta, int bl_delta, int br_delta, double power, double maxTime) {
        boolean lastRunPos = runPosition;
        ElapsedTime loopTime = new ElapsedTime();
        int fl_p;
        int fr_p;
        int bl_p;
        int br_p;
        int busyMotors;

        int max = Collections.max(Arrays.asList(Math.abs(fl_delta), Math.abs(fr_delta), Math.abs(bl_delta), Math.abs(br_delta)));
        int[] encoderSpeed = new int[4];
        int[] encoderCount = new int[4];
        byte digitalInputs = 0;
        byte motorStatus = 0;
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(driveHub);

        setDrivePower(0, 0, 0, 0);
        runToPosition(true);
        backRight.setTargetPosition(backRight.getCurrentPosition() + br_delta);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + fr_delta);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + fl_delta);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + bl_delta);

        RobotLog.i("Move");
        power /= max; // Scale power (velocity) to match distance to travel
        setDrivePower(power * Math.abs(fl_delta), power * Math.abs(bl_delta),
                power * Math.abs(fr_delta), power * Math.abs(br_delta));


        int loopCnt = 0;
        loopTime.reset();
        do {
            loopCnt++;
            busyMotors = 0;
            try {
                LynxGetBulkInputDataResponse response = command.sendReceive();
                for (int i = 0; i < 4; i++) {
                    encoderSpeed[i] = response.getVelocity(i);
                    encoderCount[i] = response.getEncoder(i);
                }
                byte[] payload = response.toPayloadByteArray();
                digitalInputs = payload[0];
                motorStatus = payload[17];
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (Exception e) {
                RobotLog.logStackTrace(e);
            }

            fl_p = frontLeft.getTargetPosition() - frontLeft.getCurrentPosition();
            fr_p = frontRight.getTargetPosition() - frontRight.getCurrentPosition();
            bl_p = backLeft.getTargetPosition() - backLeft.getCurrentPosition();
            br_p = backRight.getTargetPosition() - backRight.getCurrentPosition();
            if (Math.abs(fl_p) > 25) busyMotors++;
            if (Math.abs(fr_p) > 25) busyMotors++;
            if (Math.abs(bl_p) > 25) busyMotors++;
            if (Math.abs(br_p) > 25) busyMotors++;

            RobotLog.ii("Move", String.format("Time = %6.3f, FL = %6d, FR = %6d, BL = %6d, BR = %6d, BM = %6d", loopTime.seconds(),
                    fl_p, fr_p, bl_p, br_p, busyMotors));

            telemetry.addLine(String.format("Time = %6.3f, FL = %6d, FR = %6d, BL = %6d, BR = %6d, BM = %6d", loopTime.seconds(),
                    fl_p, fr_p, bl_p, br_p, busyMotors));
            telemetry.addLine(String.format("Encoders: FL = %6d, FR = %6d, BL = %6d, BR = %6d, BM = %6d",
                    encoderCount[0], encoderCount[1], encoderCount[2], encoderCount[3], motorStatus));
            telemetry.addLine(String.format("Cont/Sec: FL = %6d, FR = %6d, BL = %6d, BR = %6d, BM = %6d",
                    encoderSpeed[0], encoderSpeed[1], encoderSpeed[2], encoderSpeed[3], digitalInputs));
            telemetry.update();
        }
        while ((busyMotors > 1) && (loopTime.seconds() < maxTime));
        setDrivePower(0, 0, 0, 0);
        runToPosition(lastRunPos);
        return loopCnt;
    }

    public void runToPosition(boolean usePos) {
        runPosition = usePos;
        if (usePos) {
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        } else {
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    /*public void move(double fwd, double sideways, double degrees, double power, double maxTime) {
        double delta_f = TICK_FACTOR * fwd;
        double delta_s = TICK_FACTOR * sideways;
        double delta_t = (TICK_FACTOR * Math.PI * 27.17 * degrees) / 360.0; // turning radius; distance of opposing wheels

        int fl_delta = (int) (delta_f + delta_s + delta_t);
        int fr_delta = (int) (delta_f - delta_s - delta_t);
        int bl_delta = (int) (delta_f - delta_s + delta_t);
        int br_delta = (int) (delta_f + delta_s - delta_t);

        move(fl_delta, fr_delta, bl_delta, br_delta, power, maxTime);
    }*/

    public void forward(double distance_inInch, double power, double maxTime) {
        int delta;
        distance_inInch = TICK_FACTOR * distance_inInch;
        delta = (int) distance_inInch;
        setDrivePower(0, 0, 0, 0);
        runToPosition(true);
        move(delta, delta, delta, delta, power, maxTime);
    }

    public void turn(double degree, double power, double maxTime) {
        double distance_inInch = Math.PI * 20.0 * degree / 360.0; // turning radius; distance of opposing wheels
        int delta;
        distance_inInch *= 360.0 / 265.0;
        distance_inInch = TICK_FACTOR * distance_inInch;
        delta = (int) distance_inInch;
        move(delta, -delta, delta, -delta, power, maxTime);
    }

    public void setCarousel(double power) {
        carousel.setPower(power);
    }

    public void setRegula1(double power) {
        regula1.setPower(power);
    }

    public void setBaller(double power) {
        baller.setPower(power);
    }
}

    /*public void setArmVertical(double power) {
        armVertical.setPower(power);
    }

    public void setArmHorizontal(double power) {
        armHorizontal.setPower(power);
    }

    public void setCounter(double power) {
        counter.setPower(power);
    }
    //public void setArm1(double power){
    //   arm1.setPower(power);
    //}

    //public void setArm2(double power){
    //   arm2.setPower(power);
    //}


    public void setGrabbers(boolean deploy){
        grabber1.setPosition(deploy ? 0.0 : 0.75);
        grabber2.setPosition(deploy ? 0.0 : 0.75);
    }
}
    /*public void setIntakeMechanism(boolean deploy) {
        // setPlatformGrabberPosition puts in angle below
        intakeMechanismRight.setPosition(deploy ? 1.0 : .65);
        intakeMechanismLeft.setPosition(deploy ? 0-1.0 :.65);
    }
    */