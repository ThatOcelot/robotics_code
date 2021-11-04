package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ftc2021 extends BasicOpMode_Linear {
    private boolean runPosition = false;

    LynxModule driveHub = null;

    public DcMotor leftFrontDrive = null
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor carouselWheel = null;

    @Override
    public void init(HardwareMap ahwMap, Telemetry ) {
        //initializing the drive variables for configurations(USE THESE NAMES ON PHONE FOR CONFIGURATION)
        leftBackDrive = hardwareMap.get(DcMotor.class, "lbDrive");
        leftFrontDrive= hardwareMap.get(DcMotor.class, "lfDrive");
        rightBackDrive= hardwareMap.get(DcMotor.class, "rbDrive");
        rightFrontDrive= hardwareMap.get(DcMotor.class, "rfDrive");
        carouselWheel= hardwareMap.get(DcMotor.class, "carouselWheel");

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        carouselWheel.setPower(0);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void runToPosition(boolean usePos) {
        runPosition = usePos
                if (usePos) {
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    carouselWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                } else {
                    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    carouselWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
    }

    @Override
    public void loop() {


    }
}

