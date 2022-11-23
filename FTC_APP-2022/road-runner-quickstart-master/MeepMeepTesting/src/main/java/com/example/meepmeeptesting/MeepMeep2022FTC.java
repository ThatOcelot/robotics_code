package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep2022FTC {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -59, Math.toRadians(90)))




                                
                                /*TILE F5 RED SIDE{


                                /*
                                Signal location 1(
                                .lineToConstantHeading(new Vector2d(12.4, -22.3))
                                )
                                */
                                /*
                               Signal Location 2(
                               .strafeTo(new Vector2d(10.6,-60.8))
                               .splineToConstantHeading(new Vector2d(34.8,-22.3),Math.toRadians(90))
                               )
                               */
                                /*
                                Signal Location 3
                                (
                                .strafeRight(30)
                                .lineToConstantHeading(new Vector2d(56.9, -22.3))
                                )
                                */






                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}