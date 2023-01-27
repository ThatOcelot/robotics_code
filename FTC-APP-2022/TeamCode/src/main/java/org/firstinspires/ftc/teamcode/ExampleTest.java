package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Test", group = "Pushbot")

public class ExampleTest extends LinearOpMode {
    ftc2022 robot = new ftc2022();
    private ElapsedTime runtime  = new ElapsedTime();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        robot.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);





        telemetry.addData("Status","Ready to run");
        telemetry.update();


        runtime.reset();

        waitForStart();






        robot.forward(0.5,20);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds()<1.0)){
            telemetry.addData("Path","Leg 2:%2.5f S Elapsed", runtime.seconds());
            telemetry.update();



        }

    }
}
