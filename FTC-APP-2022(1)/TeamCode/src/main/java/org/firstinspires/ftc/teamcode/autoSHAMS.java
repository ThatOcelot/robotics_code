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

        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            // Strafe right for 12 inches
            robot.strafe(12, 0.5, 10,2);

            // Turn 90 degrees clockwise
            robot.turn(90);

            // Strafe left for 12 inches
            robot.strafe(-12, 0.5, 1000,2);

            // Turn 90 degrees counterclockwise
            robot.turn(-90);

            // Strafe forward for 12 inches
            robot.strafe(0, 12, 0.5, 1000);

            // Turn 180 degrees
            robot.turn(180);
        }

        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path","Leg 2:%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}
