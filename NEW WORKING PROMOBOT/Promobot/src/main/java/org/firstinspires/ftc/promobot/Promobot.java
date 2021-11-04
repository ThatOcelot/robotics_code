package org.firstinspires.ftc.promobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * PromoBot v0.1
 */
@TeleOp(name="PromoBot", group="Promobot")

public class Promobot extends OpMode{
    private enum BeltState { READY, FORWARD, REVERSE, BACKUP, STOPPED }
    private BeltState       beltState       = BeltState.READY;
    private StateMachine    LEDPattern      = null;
    private ElapsedTime     flagTimeout     = new ElapsedTime();
    private ElapsedTime     beltTimer       = new ElapsedTime();
    private ElapsedTime     motorTimer      = new ElapsedTime();
    private DcMotor         leftFrontDrive  = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightFrontDrive = null;
    private DcMotor         rightBackDrive  = null;
    private DcMotor         flagLift        = null;
    private Servo           conveyorBelt    = null;
    private Servo           LED             = null;
    private DigitalChannel  balanceSwitch   = null;
    private DigitalChannel  flagSwitch      = null;
    private double          driveTarget     = 0.0;
    private double          turnTarget      = 0.0;
    private int             flagTargetPosition = 0;
    private final double[]  LEDPatternValue = {0.256,0.2755,.2796,0.3478,0.593,0.71,0.723,0.7734};
    private double flagTime = 0;
    private boolean flagCalibrated = false;
    private boolean flagStalledUp = false;
    private boolean flagStalledDown = false;
    private int flagCurrentPosition = 0;
    private int flagCalibratedPosition = 0;
    private int flagStalledUpPosition = 99999;
    private int flagStalledDownPosition = -99999;
    private int[] flagLastPositions;
    private int flagLastPositionsCount = 0;
    private double flagLiftPower = 0.0;

    final private double flagPowerCalibrated = 0.4;
    final private double flagPowerUnCalibrated = 0.2;
    final private int flagLastPositionsCountMax = 5;
    final private int flagMaxCount = 2500;

    public void init() {
        LEDPattern = new StateMachineBuilder().numberOfStates(8).initialState(7).buildStateMachine();

         /*
         * Initialize the drive system variables.
         */
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lfDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lbDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rfDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rbDrive");
        flagLift        = hardwareMap.get(DcMotor.class, "flagLift");
        conveyorBelt    = hardwareMap.get(Servo.class, "conveyor");
        LED             = hardwareMap.get(Servo.class, "LED");
        balanceSwitch   = hardwareMap.get(DigitalChannel.class, "balanceSwitch");
        flagSwitch      = hardwareMap.get(DigitalChannel.class, "flagSwitch");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        flagLift.setDirection(DcMotorSimple.Direction.REVERSE);
        flagLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flagLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flagLift.setTargetPosition(0);
        flagLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flagLift.setPower(0.0);
        flagCalibrated = false;
        flagLastPositions = new int[flagLastPositionsCountMax];
        flagLastPositionsCount = 0;
    }

    public void start()
    {
        motorTimer.reset();
        LED.setPosition(LEDPatternValue[LEDPattern.getState()]);
    }

    public void loop(){
        //LED Code
        if (LEDPattern.stateChange(gamepad1.b)) {
            LED.setPosition(LEDPatternValue[LEDPattern.getState()]);
        }
        // telemetry.addData("LED pattern # :", LED.getPosition());

        // Candy dispenser
        boolean dispenseOnce = (gamepad1.right_trigger > 0.5);
        boolean dispenseCont = (gamepad1.right_bumper);
        BeltState oldBeltState = beltState;
        switch (beltState) {
            case READY:
                if (dispenseOnce || dispenseCont) {
                    beltState = BeltState.FORWARD;
                }
                break;
            case FORWARD:
                if ((!dispenseOnce || balanceSwitch.getState()) && !dispenseCont) {
                    beltState = BeltState.BACKUP;
                } else if (beltTimer.milliseconds() > 1000) {
                    beltState = BeltState.REVERSE;
                }
                break;
            case REVERSE:
                if ((!dispenseOnce || balanceSwitch.getState()) && !dispenseCont) {
                    beltState = BeltState.BACKUP;
                } else if (beltTimer.milliseconds() > 300) {
                    beltState = BeltState.FORWARD;
                }
                break;
            case BACKUP:
                if (beltTimer.milliseconds() > 300 && dispenseCont) {
                    beltState = BeltState.FORWARD;
                } else if (beltTimer.milliseconds() > 1000) {
                    beltState = dispenseOnce ? BeltState.STOPPED : BeltState.READY;
                }
                break;
            case STOPPED:
                if (dispenseCont) {
                    beltState = BeltState.FORWARD;
                } else if (!dispenseOnce) {
                    beltState = BeltState.READY;
                }
                break;
        }

        if (beltState != oldBeltState) {
            if (oldBeltState != BeltState.REVERSE || beltState != BeltState.BACKUP)
                beltTimer.reset();
            switch (beltState) {
                case READY:
                case STOPPED:
                    conveyorBelt.setPosition(0.5);
                    break;
                case FORWARD:
                    conveyorBelt.setPosition(0.0);
                    break;
                case REVERSE:
                case BACKUP:
                    conveyorBelt.setPosition(0.75);
                    break;
            }
        }

        // Motor Update Code
        if (motorTimer.milliseconds() > 10.0) {
            motorTimer.reset();
            //telemetry.addData("*flagTargetPosition", flagTargetPosition);

            // Flag Lift
            flagCurrentPosition = flagLift.getCurrentPosition();
            if (!flagSwitch.getState()) {
                if (!flagCalibrated) {
                    flagCalibrated = true;
                    flagCalibratedPosition = flagCurrentPosition;
                    flagStalledUp = false;
                    flagStalledDown = false;
                    flagStalledUpPosition = flagCalibratedPosition + 99999;
                    flagStalledDownPosition = flagCalibratedPosition - 99999;
                }
            } else if (!flagLift.isBusy()) {
                flagLastPositionsCount = 0;
            } else {
                if (flagLastPositionsCount == flagLastPositionsCountMax) {
                    int maxFlagPos = flagCurrentPosition;
                    int minFlagPos = flagCurrentPosition;
                    for (int i = flagLastPositionsCount - 1; i >= 0; i--) {
                        maxFlagPos = Math.max(maxFlagPos, flagLastPositions[i]);
                        minFlagPos = Math.min(minFlagPos, flagLastPositions[i]);
                    }
                    //telemetry.addData("+maxFlagPos", maxFlagPos);
                    //telemetry.addData("+minFlagPos", minFlagPos);
                    //telemetry.addData("+rangeFlagPos", maxFlagPos - minFlagPos);

                } else {
                    flagLastPositionsCount++;
                }
                for (int i = flagLastPositionsCount - 2; i >= 0; i--) {
                    flagLastPositions[i + 1] = flagLastPositions[i];
                }
                flagLastPositions[0] = flagCurrentPosition;
            }

            int flagNewTargetPosition = flagTargetPosition;
            if (gamepad1.dpad_up && !flagStalledUp) {
                flagNewTargetPosition = flagNewTargetPosition + 10;
            } else if (gamepad1.dpad_down && !flagStalledDown){
                flagNewTargetPosition = flagNewTargetPosition - 10;
            }
            flagNewTargetPosition = Range.clip(flagNewTargetPosition, flagCurrentPosition - 50, flagCurrentPosition + 50);

            if (flagCalibrated) {
                flagNewTargetPosition = Range.clip(flagNewTargetPosition, flagCalibratedPosition, flagCalibratedPosition + flagMaxCount);
            }
            //telemetry.addData("*flagNewTargetPosition", flagTargetPosition);
            //flagNewTargetPosition = Range.clip(flagNewTargetPosition, flagStalledDownPosition, flagStalledUpPosition);

            //telemetry.addData("-flagTargetPosition", flagTargetPosition);
            //telemetry.addData("-flagNewTargetPosition", flagNewTargetPosition);
            if (flagTargetPosition != flagNewTargetPosition) {
                flagTargetPosition = flagNewTargetPosition;
                if ((Math.abs(flagCurrentPosition - flagTargetPosition) < 35) || !flagCalibrated ||
                    (flagCalibrated && (flagTargetPosition < flagCurrentPosition) && (flagTargetPosition - flagCalibratedPosition) < 100) ||
                    (flagCalibrated && (flagTargetPosition > flagCurrentPosition) && (flagMaxCount - (flagTargetPosition - flagCalibratedPosition)) < 100)) {
                    flagLiftPower = flagPowerUnCalibrated;
                } else {
                    flagLiftPower = flagPowerCalibrated;
                }
                flagLift.setTargetPosition(flagTargetPosition);
                flagLift.setPower(flagLiftPower);
                flagTime = flagTimeout.seconds();
            }
            if (flagTimeout.seconds() - flagTime > 2){
                flagLift.setPower(0);
                flagLiftPower = 0.0;
            } else {
                /*
                telemetry.addData("+flagCurrentPosition", flagCurrentPosition);
                telemetry.addData("+flagTargetPosition", flagTargetPosition);
                if (Math.abs(flagTargetPosition - flagCurrentPosition) < 3) {
                    flagStalledCount = 0;
                } else if (false) { //(Math.abs(flagLastPosition - flagCurrentPosition) < 3) {
                    flagStalledCount++;
                    if (flagStalledCount > 10) {
                        flagLift.setPower(0);
                        flagTargetPosition = flagCurrentPosition;
                        flagLift.setTargetPosition(flagCurrentPosition);
                        if (flagTargetPosition > flagCurrentPosition) {
                            flagStalledUp = true;
                            flagStalledUpPosition = flagCurrentPosition - 5;
                            flagStalledDown = false;
                            flagStalledDownPosition = flagCurrentPosition - 999999;
                        } else {
                            flagStalledUp = false;
                            flagStalledUpPosition = flagCurrentPosition + 999999;
                            flagStalledDown = true;
                            flagStalledDownPosition = flagCurrentPosition + 5;
                        }
                    }
                }
                */
            }

            // Drive
            boolean slowDrive = (gamepad1.left_trigger> 0.5);

            if (Double.isNaN(driveTarget))
                driveTarget = 0.0;
            double dTarget = (Math.abs(driveTarget) > 0.01) ? driveTarget : Math.signum(driveTarget) * 0.01;
            if (dTarget == 0.0) dTarget = 0.01;
            double diff = (-gamepad1.left_stick_y) / dTarget;
            double x = Range.clip(diff, 0.8, 1.25);
            if (diff < 0 && Math.abs(driveTarget) < 0.02)
                x = Range.clip(diff, -1.5, 0.0);
            //telemetry.addData("driveTarget", driveTarget);
            //telemetry.addData("dTarget", dTarget);
            //telemetry.addData("diff", diff);
            //telemetry.addData("x", x);
            driveTarget = dTarget * x;
            if (Math.abs(driveTarget) < 0.01)
                driveTarget = 0.0;
            diff = (gamepad1.right_stick_x/(slowDrive ? 1.0 : 2.5)) - turnTarget;
            turnTarget += Range.clip(diff, -0.05, 0.05);

            double drive = driveTarget;
            double turn  = turnTarget;
            double mag = Math.abs(drive) + Math.abs(turn);
            if (mag >= 1.0) {
                drive /= mag;
                turn /= mag;
            }
            double leftPower  = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);
            double speedDelta = slowDrive ? 0.3 : 1.0;
            leftFrontDrive.setPower(-leftPower * speedDelta/2);
            leftBackDrive.setPower(-leftPower * speedDelta/2);
            rightFrontDrive.setPower(-rightPower * speedDelta/2);
            rightBackDrive.setPower(-rightPower * speedDelta/2);
        }

        telemetry.addData("flagLiftPower", flagLiftPower);
        telemetry.addData("flagCurrentPosition", flagCurrentPosition - flagCalibratedPosition);
        telemetry.addData("flagTargetPosition", flagTargetPosition - flagCalibratedPosition);
//        telemetry.addData("flagCalibratedPosition", flagCalibratedPosition);
//        telemetry.addData("flagStalledUpPosition", flagStalledUpPosition);
//        telemetry.addData("flagStalledDownPosition", flagStalledDownPosition);
        //telemetry.addData("flagStalledCount", flagStalledCount);
        telemetry.addData("driveTarget", driveTarget);
        telemetry.addData("turnTarget", turnTarget);
//        telemetry.addData("flagSwitch", flagSwitch.getState());
        telemetry.update();

    }

    //*********************************************************************************************
    //*********************************   Basic Drive Functions   *********************************
    //*********************************************************************************************

    void move(double left, double right){
        leftFrontDrive.setPower(left);
        leftBackDrive.setPower(left);
        rightFrontDrive.setPower(right);
        rightBackDrive.setPower(right);
    }

    void setLEDPattern(int patternNum){
        LED.setPosition(0.224234+(0.005383*patternNum));
    }
}

