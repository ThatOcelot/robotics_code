package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="RobotTest 2.0", group="Test")
public class teleOpJAMAL extends LinearOpMode {
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
        double fb_mult = 0.8;
        double st_mult = 0.8;
        double lr_mult = 0.8;
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;
        // Left Y = Fwd / Back
        fl = -gamepad1.left_stick_y * fb_mult;
        bl = -gamepad1.left_stick_y * fb_mult;
        fr = -gamepad1.left_stick_y * fb_mult;
        br = -gamepad1.left_stick_y * fb_mult;
        // Left X = Strafe L/R
        fl += gamepad1.left_stick_x * st_mult;
        br += gamepad1.left_stick_x * st_mult;
        fr += -gamepad1.left_stick_x * st_mult;
        bl += -gamepad1.left_stick_x * st_mult;
        // left and right turn
        fl += gamepad1.right_stick_x * lr_mult;
        bl += gamepad1.right_stick_x * lr_mult;
        fr += -gamepad1.right_stick_x * lr_mult;
        br += -gamepad1.right_stick_x * lr_mult;
        double mx = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (mx < 0.2) {
            fl = fr = bl = br = 0.0;
        } else if (mx > 1.0) {
            fl *= 1.0 / mx;
            fr *= 1.0 / mx;
            bl *= 1.0 / mx;
            br *= 1.0 / mx;
        }
        robot.setDrivePower(fl, bl, fr, br);

        if(gamepad2.a)
        {
            robot.setEnrique(-0.8);

        }
        else if (gamepad2.y)
        {
            robot.setEnrique(0.8);
        }
        else robot.setEnrique(0);

        if(gamepad2.x)
        {
            robot.setDjkhalid(true);
        }
        else if (gamepad2.b)
        {
            robot.setDjkhalid(false);
        }



    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */




        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Vertical", robot.armHorizontal);
        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Version", "1.8");
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
