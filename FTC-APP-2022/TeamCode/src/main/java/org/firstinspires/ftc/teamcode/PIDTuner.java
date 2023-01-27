package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous
public class PIDTuner extends LinearOpMode {

    ftc2022 robot = new ftc2022();
    double kp, ki, kd;
    ArrayList<Double> positionList = new ArrayList<>();
    ArrayList<Double> timeList = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        kp = 0.46;
        ki = 0.01;
        kd = 0.1;

        ElapsedTime timer = new ElapsedTime();
        robot.init(hardwareMap, telemetry);

        waitForStart();

        for (int i = 0; i < 2; i++) {
            // set the target position to be 10 rotations
            int targetPosition = 47 * 40;
            int currentPosition = 0;
            double currentTime = 0;
            double error = 0;
            double integral = 0;
            timer.reset();
            // set the target position of example right Front motor
            robot.rightFront.setTargetPosition(targetPosition);
            robot.rightRear.setTargetPosition(targetPosition);
            robot.leftFront.setTargetPosition(targetPosition);
            robot.leftRear.setTargetPosition(targetPosition);

            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() && robot.rightFront.isBusy() && robot.rightRear.isBusy() && robot.leftRear.isBusy() && robot.leftFront.isBusy()) {
                // calculate the error, integral, and derivative
                error = targetPosition - robot.rightFront.getCurrentPosition();

                integral += error * (timer.time() - currentTime);
                double derivative = (error - (targetPosition - currentPosition)) / (timer.time() - currentTime);

                // set the power of the motors using the PID values
                robot.rightFront.setPower(kp * error + ki * integral + kd * derivative);
                robot.rightRear.setPower(kp * error + ki * integral + kd * derivative);
                robot.leftFront.setPower(kp * error + ki * integral + kd * derivative);
                robot.leftRear.setPower(kp * error + ki * integral + kd * derivative);


                // display the kp value
                telemetry.addData("kp", kp);
                // check if the
// check if the kp value is changed
                if (gamepad1.dpad_up) {
                    kp += 0.01;
                    telemetry.addData("Kp value: ", kp);
                    telemetry.update();
                }
                if (gamepad1.dpad_down) {
                    kp -= 0.01;
                    telemetry.addData("Kp value: ", kp);
                    telemetry.update();
                }
                if (gamepad1.dpad_left) {
                    kp -= 0.1;
                    telemetry.addData("Kp value: ", kp);
                    telemetry.update();
                }
                if (gamepad1.dpad_right) {
                    kp += 0.1;
                    telemetry.addData("Kp value: ", kp);
                    telemetry.update();
                }
// add the current position and time to the lists
                positionList.add((double) currentPosition);
                timeList.add(currentTime);
// display the current position and time
                telemetry.addData("Position", currentPosition);
                telemetry.addData("Time", currentTime);
                telemetry.update();
            }
// stop the motors
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);
//reverse direction of the robot
            targetPosition = -targetPosition;
        }
    }
}

