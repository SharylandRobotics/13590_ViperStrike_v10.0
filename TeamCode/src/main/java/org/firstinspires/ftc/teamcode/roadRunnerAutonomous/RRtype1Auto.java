package org.firstinspires.ftc.teamcode.roadRunnerAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RRactions;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;

import java.lang.Math;

@Config
@Autonomous(name = "RR Specimen Test", group = "RoadRunner")
public class RRtype1Auto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -61.25, Math.toRadians(90));
        Pose2d rungPose = new Pose2d(9.5, -40, Math.toRadians(90));

        Pose2d sample1Pose = new Pose2d(35, -34, Math.toRadians(50));
        Pose2d sample2Pose = new Pose2d(60, -34, Math.toRadians(60));
        Pose2d sample3Pose = new Pose2d(40, -34, Math.toRadians(50));

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

        Action leg3 = drive.actionBuilder(backupPose)
                .setTangent(0)
                .lineToXLinearHeading(45, Math.toRadians(45))
                .build();

        Action backup2 = drive.actionBuilder(rungPose)
                .setTangent(Math.PI/2)
                .lineToY(-45)
                .build();


        robot.init(true);

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                leg1,
                                elbow.elbowToDeg(112),
                                extension.extenderToInch(8.5),
                                axial.rotateAxial(robot.CLAW_UP)
                        ),
                        axial.rotateAxial(robot.CLAW_DOWN),
                        elbow.elbowToDeg(60),

                        pinch.openClaw(),
                        new ParallelAction(
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(0),
                                backup2
                        ),
                        new ParallelAction(
                                leg3,
                                elbow.elbowToDeg(24)
                        ),
                        new ParallelAction(
                                yaw.rotateClaw(0.315),
                                extension.extenderToInch(8.5)
                        )
                        // leg 2 actions

                )
        );
    }
}
