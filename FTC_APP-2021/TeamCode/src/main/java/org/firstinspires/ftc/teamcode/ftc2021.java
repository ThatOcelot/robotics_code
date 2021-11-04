package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ftc2021 extends OpMode {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor carouselWheel;

    @Override
    public void init() {
        //initializing the drive variables for configurations(USE THESE NAMES ON PHONE FOR CONFIGURATION)
        leftBackDrive = hardwareMap.get(DcMotor.class, "lbDrive");
        leftFrontDrive= hardwareMap.get(DcMotor.class, "lfDrive");
        rightBackDrive= hardwareMap.get(DcMotor.class, "rbDrive");
        rightFrontDrive= hardwareMap.get(DcMotor.class, "rfDrive");
        carouselWheel= hardwareMap.get(DcMotor.class, "carouselWheel");

    }

    @Override
    public void loop() {

    }
}

