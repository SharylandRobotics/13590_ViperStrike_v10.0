package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep2 {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(750);

        Pose2d initialPose = new Pose2d(-9.5, -64.5, Math.toRadians(90));
        Pose2d rungPose = new Pose2d(-9.5, -43.25, Math.toRadians(90));

        Pose2d sample1Pose = new Pose2d(-48, -33.75, Math.toRadians(90));
        Pose2d drop1Pose = new Pose2d(-55,-55, Math.toRadians(45));

        Pose2d sample2Pose = new Pose2d(-57, -34, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(-56, -38, Math.toRadians(130));

        Pose2d parkSpot = new Pose2d(-24, 0, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .lineToY(rungPose.position.y)

                .setTangent(Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x)) // just the atan (y2-y1 / x2-x1)
                .lineToX(sample1Pose.position.x)

                .setTangent(Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .lineToXLinearHeading(drop1Pose.position.x, drop1Pose.heading)

                .setTangent(Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .lineToYLinearHeading(sample2Pose.position.y, sample2Pose.heading)

                .setTangent(Math.atan2(drop1Pose.position.y - sample2Pose.position.y, drop1Pose.position.x - sample2Pose.position.x))
                .lineToYLinearHeading(drop1Pose.position.y, drop1Pose.heading)

                .setTangent(Math.atan2(sample3Pose.position.y - drop1Pose.position.y, sample3Pose.position.x - drop1Pose.position.x))
                .lineToYLinearHeading(sample3Pose.position.y, sample3Pose.heading)

                .setTangent(Math.atan2(drop1Pose.position.y - sample3Pose.position.y, drop1Pose.position.x - sample3Pose.position.x))
                .lineToXLinearHeading(drop1Pose.position.x, drop1Pose.heading)

                .setTangent(Math.atan2(parkSpot.position.y - drop1Pose.position.y, parkSpot.position.x - drop1Pose.position.x))
                .splineToLinearHeading(parkSpot, Math.toRadians(40))

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}