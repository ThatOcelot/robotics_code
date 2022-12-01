package org.firstinspires.ftc.teamcode.drive.opmode.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class teleOP2022 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )


            );

            // An Enum is used to represent lift states.
                // (This is one thing enums are designed to do)
            
            //Makes the carousel spin clockwise when pressing dpad right, and spin counterclockwise
            //when pressing dpad left
            if (gamepad2.dpad_right)
            {
                drive.setSpin(0.7);
            }
            else if (gamepad2.dpad_left)
            {
                drive.setSpin(-0.7);
            }
            else drive.setSpin(0);

            //Makes vertical slides go upwards when pressing y, and downwards when pressing a
            if (gamepad2.y)
            {
                drive.setVert(1.0);
            }
            else if (gamepad2.a)
            {
                drive.setVert(-0.7);
            }
            else drive.setVert(0);

            //Makes horizontal slide go forward when pressing x, and backward when pressing b
            if (gamepad2.x)
            {
                drive.setGrab(true);
            }
            else if (gamepad2.b)
            {
                drive.setGrab(false);
            }





            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}