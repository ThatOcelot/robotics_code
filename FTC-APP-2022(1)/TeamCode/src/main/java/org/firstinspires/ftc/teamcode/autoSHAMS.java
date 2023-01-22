package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "autonomous", group = "Pushbot")

public class autoSHAMS extends LinearOpMode {
    ftc2022 robot = new ftc2022();
    private ElapsedTime runtime  = new ElapsedTime();

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";







    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);

            }

            @Override
            public void onError(int errorCode) {}
        });

        robot.init(hardwareMap, telemetry);



        telemetry.addData("Status","Ready to run");
        telemetry.addData("Position", sleeveDetection.getPosition());
        telemetry.update();

        runtime.reset();

        waitForStart();

        while (opModeIsActive() && (runtime.seconds() < 30.0)) {

            if (sleeveDetection.getPosition()== SleeveDetection.ParkingPosition.LEFT)
            {
                robot.strafeForward(10);
                robot.turn(-90);
                robot.strafeForward(10);
                robot.turn(90);
                robot.strafeForward(10);

            }

            if (sleeveDetection.getPosition()==SleeveDetection.ParkingPosition.CENTER)
            {
                robot.strafeForward(20);

            }

            if (sleeveDetection.getPosition()== SleeveDetection.ParkingPosition.RIGHT)
            {
                robot.strafeForward(10);
                robot.turn(90);
                robot.strafeForward(10);
                robot.turn(-90);
                robot.strafeForward(10);

            }









        }

        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path","Leg 2:%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}
