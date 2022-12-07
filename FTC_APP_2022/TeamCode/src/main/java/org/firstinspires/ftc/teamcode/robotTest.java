package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="RobotTest 2.0", group="Test")
public class robotTest  extends LinearOpMode {
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


        if (gamepad1.a) {
            robot.setCarousel(1.0);
        } else if (gamepad1.b) {
            robot.setCarousel(-1.0);
        } else robot.setCarousel(0.0);

        if (gamepad2.square) {
            robot.setRegula1((1.0));
        } else if (gamepad2.circle) {
                robot.setRegula1(-1.0);
            } else robot.setRegula1(0.0);

        if (gamepad2.triangle) {
            robot.setBaller(1.0);
        } else if (gamepad2.cross) {
            robot.setBaller(-1.0);
        } else robot.setBaller(0.0);

        /*if(gamepad2.dpad_left){
            robot.setGrabbers(false);
        } else robot.setGrabbers(true);

        if(gamepad2.dpad_up){
            robot.setArmHorizontal(1.0);
        }else if(gamepad2.dpad_down){
            robot.setArmHorizontal(-1.0);
        }else robot.setArmHorizontal(0.0);

        if (gamepad2.a){
            robot.setArmVertical(1.0);
        }else if(gamepad2.y){
            robot.setArmVertical(-1.0);
        }else robot.setArmVertical(0.0);

        if (gamepad2.b){
            robot.setCounter(1.0);
        }else if(gamepad2.x){
            robot.setCounter(-1.0);
        }else robot.setCounter(0.0);
    }*/
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        boolean tapeRetracting = false;

        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Vertical", robot.armHorizontal);
        telemetry.addData("Status", "Waiting for Start");
        telemetry.addData("Version", "1.6");
        telemetry.update();

        //robot.grabber2.setPosition(0.0);
        //robot.grabber1.setPosition(0.0);


        /*StateMachine armVerticalHalf = new StateMachineBuilder().numberOfStates(2).initialState(0).buildStateMachine();
        armVerticalHalf.stateChange(gamepad1.dpad_up);
        if (armVerticalHalf.getState() == 1) {
            robot.armVertical.setPower(1.0);
        }*/
      /*  if(gamepad1.y){
            robot.setArm1(1.0);
        }else if(gamepad1.x){
            robot.setArm1((-1.0));
        } else if (gamepad1.b) {
            robot.setArm1((0.0));
        }
        if(gamepad1.y) {
            robot.setArm2(1.0);
            ;
        }else if(gamepad1.x){
            robot.setArm2(-1.0);
        }else if (gamepad1.b) {
            robot.setArm2((0.0));

        }
*/

        //moves the carousel motor to the right when pressing A, moves to the  left when pressing B

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

/**
 * This if statement allows us to use the grabbers on the back of the robot.
 */


/**
 *This boolean allows us to control the horizontal arm on the robot with movements of the right stick of controller 2.
 * It also stops the arm when the joystick is not being moved.
 */


/**
 * This boolean allows us to control the vertical arm on the robot with movements of the left stick of controller 2.
 * It also stops the arm when the joystick is not being moved.
 */

/**
 * This if statement states that controls the tape measure with the left and right triggers on the controller.
 */

