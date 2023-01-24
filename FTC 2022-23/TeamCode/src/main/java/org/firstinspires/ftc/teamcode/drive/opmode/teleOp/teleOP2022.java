package org.firstinspires.ftc.teamcode.drive.opmode.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        // Set the drive motors to run without encoders
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            //Drive the robot using the left stick for forward/backward and strafing, and the right stick for rotation
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            // Control a motor named 'enrique' using the A and Y buttons on gamepad2
            /*if (gamepad2.a) {
                drive.setEnrique(0.8);
                drive.enrique.setTargetPosition(drive.enrique.getCurrentPosition() + 200);
            } else if (gamepad2.y) {
                drive.setEnrique(0.8);
                drive.enrique.setTargetPosition(drive.enrique.getCurrentPosition() - 200);
            }
            else {
                drive.setEnrique(0);
            }*/

            // Control a motor named 'djkhalid' using the X and B buttons on gamepad2
            if (gamepad2.x) {
                drive.setDjkhalid(true);
            } else if (gamepad2.b) {
                drive.setDjkhalid(false);
            }
            else {
                drive.setDjkhalid(false);
            }

            drive.update();

            // Send telemetry data to the driver station
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.
                    getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}