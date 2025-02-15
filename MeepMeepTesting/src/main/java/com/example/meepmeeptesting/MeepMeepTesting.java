package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[]){
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9.5, -61.25, Math.toRadians(90)))
                        .lineTo(new Vector2d(9.5, -40))
                        .setTangent(0.230656436) // just the atan (y2-y1 / x2-x1)
                        .splineToLinearHeading(new Pose2d(35, -34, Math.toRadians(45)), 0.230656436)
                        // grab sample
                        .setTangent(Math.atan2(-11, 11))
                        .splineToLinearHeading(new Pose2d(46, -45, Math.toRadians(-45)), Math.atan2(-11, 11))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        .setTangent(Math.atan2(11, 15))
                        .splineToLinearHeading(new Pose2d(61, -34, Math.toRadians(60)), Math.atan2(11, 15))
                        // grab sample
                        .setTangent(Math.atan2(-13, -8))
                        .splineToLinearHeading(new Pose2d(53, -45, Math.toRadians(-45)), Math.atan2(-13, -8))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        .setTangent(Math.atan2(11, -5))
                        .splineToLinearHeading(new Pose2d(48, -34, Math.toRadians(45)), Math.atan2(11, -5))
                        // grab sample
                        .setTangent(Math.atan2(-14.5, -11.5))
                        .splineToLinearHeading(new Pose2d(36.5, -48.5, Math.toRadians(-45)), Math.atan2(-14.5, -11.5))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        // repeat to score ->
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