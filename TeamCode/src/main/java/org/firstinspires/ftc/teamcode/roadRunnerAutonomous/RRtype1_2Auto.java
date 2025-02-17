package org.firstinspires.ftc.teamcode.roadRunnerAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RRactions;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;

import java.lang.Math;

@Config
@Autonomous(name = "RR Bucket Test", group = "RoadRunner")
public class RRtype1_2Auto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-9.5, -61.25, Math.toRadians(90));
        Pose2d rungPose = new Pose2d(-9.5, -40, Math.toRadians(90));

        Pose2d sample1Pose = new Pose2d(-48, -34, Math.toRadians(90));
        Pose2d drop1Pose = new Pose2d(-55,-55, Math.toRadians(45));

        Pose2d sample2Pose = new Pose2d(-59, -34, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(-59, -34, Math.toRadians(130));

        Pose2d parkSpot = new Pose2d(-24, 0, Math.toRadians(90));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRactions.Elbow elbow = actionLib.new Elbow(hardwareMap);
        RRactions.Extender extension = actionLib.new Extender(hardwareMap);
        RRactions.Pincher pinch = actionLib.new Pincher(hardwareMap);
        RRactions.Yaw yaw = actionLib.new Yaw(hardwareMap);
        RRactions.Axial axial = actionLib.new Axial(hardwareMap);

        Action score1 = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .lineToY(-40)
                .waitSeconds(1)
                .build();

        Action grabSample1 = drive.actionBuilder(rungPose)
                .setTangent(Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x)) // just the atan (y2-y1 / x2-x1)
                .splineToLinearHeading(sample1Pose, Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x))
                .build();

        Action dropSample1 = drive.actionBuilder(sample1Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .splineToLinearHeading(drop1Pose, Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .build();

        Action grabSample2 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .splineToLinearHeading(sample2Pose, Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .build();

        Action dropSample2 = drive.actionBuilder(sample2Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample2Pose.position.y, drop1Pose.position.x - sample2Pose.position.x))
                .splineToLinearHeading(drop1Pose, Math.atan2(drop1Pose.position.y - sample2Pose.position.y, drop1Pose.position.x - sample2Pose.position.x))
                .build();

        Action grabSample3 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample3Pose.position.y - drop1Pose.position.y, sample3Pose.position.x - drop1Pose.position.x))
                .splineToLinearHeading(sample3Pose, Math.atan2(sample3Pose.position.y - drop1Pose.position.y, sample3Pose.position.x - drop1Pose.position.x))
                .build();

        Action dropSample3 = drive.actionBuilder(sample3Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample3Pose.position.y, drop1Pose.position.x - sample3Pose.position.x))
                .splineToLinearHeading(drop1Pose, Math.atan2(drop1Pose.position.y - sample3Pose.position.y, drop1Pose.position.x - sample3Pose.position.x))
                .build();

        Action toPark = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(parkSpot.position.y - drop1Pose.position.y, parkSpot.position.x - drop1Pose.position.x))
                .splineToLinearHeading(parkSpot, Math.toRadians(60))
                        .build();

        robot.init(true);

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                score1,
                                elbow.elbowToDeg(112),
                                extension.extenderToInch(8.5),
                                axial.rotateAxial(robot.CLAW_UP)
                        ),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),

                        pinch.openClaw(),
                        new ParallelAction(
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(0)
                        ),

                        grabSample1,
                        new ParallelAction(
                                elbow.elbowToDeg(15),
                                yaw.rotateClaw(0.5),
                                extension.extenderToInch(8.5)
                        ),
                        axial.rotateAxial(0.35),
                        sleepAction(1000),
                        pinch.closeClaw(),
                        sleepAction(1000),

                        new ParallelAction(
                                dropSample1,
                                elbow.elbowToDeg(120),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),

                        new ParallelAction(
                                grabSample2,
                                elbow.elbowToDeg(15),
                                yaw.rotateClaw(0.315),
                                extension.extenderToInch(8.5)
                        ),
                        axial.rotateAxial(0.35),
                        sleepAction(1000),
                        pinch.closeClaw(),
                        sleepAction(1000),

                        new ParallelAction(
                                dropSample2,
                                elbow.elbowToDeg(120),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),

                        new ParallelAction(
                                grabSample3,
                                elbow.elbowToDeg(15),
                                yaw.rotateClaw(0.315),
                                extension.extenderToInch(8.5)
                        ),
                        axial.rotateAxial(0.35),
                        sleepAction(1000),
                        pinch.closeClaw(),
                        sleepAction(1000),

                        new ParallelAction(
                                dropSample3,
                                elbow.elbowToDeg(120),
                                extension.extenderToInch(8.5),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),

                        sleepAction(1000),


                        new ParallelAction(
                                toPark,
                                elbow.elbowToDeg(0),
                                extension.extenderToInch(0)
                        )

                        // leg 2 actions

                )
        );
    }
    private Action sleepAction(long milliseconds) {
        return (TelemetryPacket packet) -> {
            sleep(milliseconds);
            return false;
        };
    }
}
