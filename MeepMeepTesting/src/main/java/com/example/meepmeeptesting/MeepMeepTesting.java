package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(750);

        Pose2d initialPose = new Pose2d(9.5, -64.5, Math.toRadians(90)); // subtracted 3.25 in y
        Pose2d rungPose = new Pose2d(9.5, -43.25, Math.toRadians(90)); // subtracted 3.25 in y

        Pose2d sample1Pose = new Pose2d(48, -34.3, Math.toRadians(90));
        Pose2d drop1Pose = new Pose2d(58,-48, Math.toRadians(90));

        Pose2d sample2Pose = new Pose2d(58, -34, Math.toRadians(90));
        Pose2d drop2Pose = new Pose2d(58, -48, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(58, -34, Math.toRadians(40));
        Pose2d drop3Pose = new Pose2d(48, -48, Math.toRadians(90));

        Pose2d pickupPose = new Pose2d(35.5, drop3Pose.position.y, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .lineToY(rungPose.position.y)
                .setTangent(Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x)) // just the atan (y2-y1 / x2-x1)
                                .lineToX(sample1Pose.position.x)
                //.splineToLinearHeading(sample1Pose, Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x))
                .setTangent(Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                                .lineToX(drop1Pose.position.x)
                //.splineToLinearHeading(drop1Pose, Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .setTangent(Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                                .lineToY(sample2Pose.position.y)
                //.splineToLinearHeading(sample2Pose, Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .setTangent(Math.atan2(drop2Pose.position.y - sample2Pose.position.y, drop2Pose.position.x - sample2Pose.position.x))
                                .lineToY(drop2Pose.position.y)
                //.splineToLinearHeading(drop2Pose, Math.atan2(drop2Pose.position.y - sample2Pose.position.y, drop2Pose.position.x - sample2Pose.position.x))
                .setTangent(Math.atan2(sample3Pose.position.y - drop2Pose.position.y, sample3Pose.position.x - drop2Pose.position.x))
                                .lineToYLinearHeading(sample3Pose.position.y, sample3Pose.heading)
                .setTangent(Math.atan2(drop3Pose.position.y - sample3Pose.position.y, drop3Pose.position.x - sample3Pose.position.x))
                                .lineToXLinearHeading(drop3Pose.position.x, drop3Pose.heading)
                //.splineToLinearHeading(drop3Pose, Math.atan2(-14.5, -11.5))
                .setTangent(Math.atan2(rungPose.position.y - drop3Pose.position.y, rungPose.position.x - drop3Pose.position.x))
                                .lineToX(rungPose.position.x)
                //.splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(90)), Math.atan2(rungPose.position.y - drop3Pose.position.y, rungPose.position.x - drop3Pose.position.x))
                .setTangent(Math.atan2(pickupPose.position.y - rungPose.position.y, pickupPose.position.x - rungPose.position.x))
                        .lineToXLinearHeading(pickupPose.position.x, pickupPose.heading)
                .setTangent(Math.atan2(rungPose.position.y - pickupPose.position.y, rungPose.position.x - pickupPose.position.x))
                        .lineToX(rungPose.position.x)
                .setTangent(Math.atan2(pickupPose.position.y - rungPose.position.y, pickupPose.position.x - rungPose.position.x))
                .lineToXLinearHeading(pickupPose.position.x, pickupPose.heading)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}