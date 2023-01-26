package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="MOTOR DEBUGGER", group="Test")
public class MotorTester extends LinearOpMode {
    ftc2022 robot = new ftc2022();    // Use a Pushbot's hardware

    void tankMode() {
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;
        if (gamepad1.left_stick_y < -0.2 || 0.2 < gamepad1.left_stick_y) {
            fl = -gamepad1.left_stick_y;
            bl = -gamepad1.left_stick_y;
        } else if (gamepad1.left_stick_x < -0.2 || 0.2 < gamepad1.left_stick_x) {
            fl = gamepad1.left_stick_x;
            bl = -gamepad1.left_stick_x;
        }
        if (gamepad1.right_stick_y < -0.2 || 0.2 < gamepad1.right_stick_y) {
            fr = -gamepad1.right_stick_y;
            br = -gamepad1.right_stick_y;
        } else if (gamepad1.right_stick_x < -0.2 || 0.2 < gamepad1.right_stick_x) {
            fr = gamepad1.right_stick_x;
            br = -gamepad1.right_stick_x;
        }
        robot.setDrivePower(fl, bl, fr, br);

    }

    // left joystick is for straight movement
    // right joystick is for left and right turn
    void omniMode() {


        if (gamepad1.y)
        {
            robot.setDrivePower(1.0,0,0,0);
        }
        else if (gamepad1.dpad_up)
        {
            robot.setDrivePower(-1.0,0,0,0);
        }

        if (gamepad1.x)
        {
            robot.setDrivePower(0,0,1.0,0);
        }
        else if (gamepad1.dpad_left)
        {
            robot.setDrivePower(0,0,-1.0,0);
        }
        if (gamepad1.b)
        {
            robot.setDrivePower(0,1,0,0);
        }
        else if (gamepad1.dpad_right)
        {
            robot.setDrivePower(0,-1.0,0,0);
        }

        if (gamepad1.a)
        {
            robot.setDrivePower(0,0,0,1.0);
        }
        else if (gamepad1.dpad_down)
        {
            robot.setDrivePower(0,0,0,-1.0);
        }



    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Front Left Motor","Y");
        telemetry.addData("Front right Motor", "X");
        telemetry.addData("Rear Left motor", "B");
        telemetry.addData("Rear Right Motor", "A");
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {
            if (false) {
                tankMode();
            } else if (true) {
                omniMode();
            }
        }

    }
}
