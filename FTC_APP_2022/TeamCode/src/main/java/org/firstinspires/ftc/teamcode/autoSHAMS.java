package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "autonomous", group = "Pushbot")

public class autoSHAMS extends LinearOpMode {
    ftc2022 robot = new ftc2022();
    private ElapsedTime runtime  = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);



        telemetry.addData("Status","Ready to run");
        telemetry.update();


        runtime.reset();

        waitForStart();

        robot.frontLeft.setPower(FORWARD_SPEED);
        robot.backLeft.setPower(FORWARD_SPEED);
        robot.frontRight.setPower(FORWARD_SPEED);
        robot.backRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()<2.0)) {
            telemetry.addData("Path","Leg 1:%2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }

        robot.frontLeft.setPower(-TURN_SPEED);
        robot.backLeft.setPower(-TURN_SPEED);
        robot.frontRight.setPower(TURN_SPEED);
        robot.backRight.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()<1.0)){
            telemetry.addData("Path","Leg 2:%2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }

    }
}
