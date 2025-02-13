package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[]){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9.5, -61.25, Math.toRadians(90)))
                        .lineTo(new Vector2d(9.5, -40))
                        .lineToLinearHeading(new Pose2d(35, -34, Math.toRadians(45)))
                        // grab sample
                        .lineToLinearHeading(new Pose2d(46, -34, Math.toRadians(-45)))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        .lineToLinearHeading(new Pose2d(60, -34, Math.toRadians(60)))
                        // grab sample
                        .lineToLinearHeading(new Pose2d(46, -34, Math.toRadians(-45)))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        .lineToLinearHeading(new Pose2d(40, -34, Math.toRadians(45)))
                        // grab sample
                        .lineToLinearHeading(new Pose2d(36, -48, Math.toRadians(-45)))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        // repeat to score ->
                        .lineToConstantHeading(new Vector2d(32, -52))
                        // grab specimen (on ground)
                        .lineToLinearHeading(new Pose2d(9, -40, Math.toRadians(90)))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}