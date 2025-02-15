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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(21.5, -54.75, Math.toRadians(90)))
                        .setTangent(Math.PI/3)
                        .splineToConstantHeading(new Vector2d(24, 0), Math.toRadians(180))
                        // 72 - 17.25
                        /*
                        .setTangent(Math.PI/2)
                        .waitSeconds(1)
                        .lineTo(new Vector2d(9.5, -40))
                        .lineToLinearHeading(new Pose2d(48, -34, Math.toRadians(90)))
                        // grab sample
                        .lineToConstantHeading(new Vector2d(59, -48))
                        .addDisplacementMarker(() ->{
                            // turn elbow around, drop sample
                        })
                        .lineTo(new Vector2d(59, -34))
                        // grab sample
                        .lineToLinearHeading(new Pose2d(59, -48, Math.toRadians(90)))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        .lineToLinearHeading(new Pose2d(59, -34, Math.toRadians(45)))
                        // grab sample
                        .lineToLinearHeading(new Pose2d(48, -48, Math.toRadians(90)))
                        .addDisplacementMarker(() ->{
                            // drop sample
                        })
                        // drive up to prevent penalty
                        .lineTo(new Vector2d(48, -44))
                        .addDisplacementMarker(() ->{
                            // extend & grab 1st specimen
                        })
                        // turn elbow
                        // score first specimen
                        .lineToLinearHeading(new Pose2d(9, -40, Math.toRadians(90)))
                        // repeat to score ->
                        .lineToConstantHeading(new Vector2d(34, -54))
                        // grab specimen (on ground)
                        .lineToLinearHeading(new Pose2d(9, -40, Math.toRadians(90)))
                        // extend & score

                         */



                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}