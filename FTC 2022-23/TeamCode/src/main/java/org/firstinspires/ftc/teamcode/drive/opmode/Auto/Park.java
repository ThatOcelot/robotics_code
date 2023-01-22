package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Signal Sleeve Test")
public class Park extends LinearOpMode {


    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, -5);
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(startPose);

        Trajectory P1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12.4, -22.3))
                .build();

        Trajectory P2 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(10.6,-60.8))
                .splineToConstantHeading(new Vector2d(34.8,-22.3),Math.toRadians(90))
                .build();

        Trajectory P3 = drive.trajectoryBuilder(startPose)
                .strafeRight(30)
                .lineToConstantHeading(new Vector2d(56.9, -22.3))
                .build();




        while (!isStarted()) {

            //.followTrajectorySequence(drive ->
                   // drive.trajectorySequenceBuilder(new Pose2d(35, -59, Math.toRadians(90)))





                            /*TILE F5 RED SIDE{


                            /*
                            Signal location 1(
                            .lineToConstantHeading(new Vector2d(12.4, -22.3))
                            )
                            */
                            /*
                           Signal Location 2(

                           )
                           */
                            /*
                            Signal Location 3
                            (

                            )
                            */










            if (sleeveDetection.getPosition()== SleeveDetection.ParkingPosition.LEFT)
            {
                drive.followTrajectory(P1);


            }


            if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER)
            {
                drive.followTrajectory(P2);

            }


            if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT)
            {
                drive.followTrajectory(P3);

            }
            waitForStart();
            timer.reset();
            while(timer.seconds()<3) drive.update();



            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
    }
}
