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
@Autonomous(name = "RR Specimen Test", group = "RoadRunner")
public class RRtype1Auto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -64.5, Math.toRadians(90)); // subtracted 3.25 in y
        Pose2d rungPose = new Pose2d(9.5, -43.25, Math.toRadians(90)); // subtracted 3.25 in y

        Pose2d sample1Pose = new Pose2d(48, -33.75, Math.toRadians(90));
        Pose2d drop1Pose = new Pose2d(57,-48, Math.toRadians(90));

        Pose2d sample2Pose = new Pose2d(57, -34, Math.toRadians(90));
        Pose2d drop2Pose = new Pose2d(57, -48, Math.toRadians(90));

        Pose2d sample3Pose = new Pose2d(56, -38, Math.toRadians(45));
        Pose2d drop3Pose = new Pose2d(48, -48, Math.toRadians(90));

        Pose2d pickupPose = new Pose2d(35.5, drop3Pose.position.y - 3, Math.toRadians(90));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRactions.Elbow elbow = actionLib.new Elbow(hardwareMap);
        RRactions.Extender extension = actionLib.new Extender(hardwareMap);
        RRactions.Pincher pinch = actionLib.new Pincher(hardwareMap);
        RRactions.Yaw yaw = actionLib.new Yaw(hardwareMap);
        RRactions.Axial axial = actionLib.new Axial(hardwareMap);

        Action score1 = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .lineToY(rungPose.position.y)
                .waitSeconds(1)
                .build();

        Action grabSample1 = drive.actionBuilder(rungPose)
                .setTangent(Math.atan2(sample1Pose.position.y - rungPose.position.y, sample1Pose.position.x - rungPose.position.x)) // just the atan (y2-y1 / x2-x1)
                .lineToX(sample1Pose.position.x)
                .build();

        Action dropSample1 = drive.actionBuilder(sample1Pose)
                .setTangent(Math.atan2(drop1Pose.position.y - sample1Pose.position.y, drop1Pose.position.x - sample1Pose.position.x))
                .lineToX(drop1Pose.position.x)
                .build();

        Action grabSample2 = drive.actionBuilder(drop1Pose)
                .setTangent(Math.atan2(sample2Pose.position.y - drop1Pose.position.y, sample2Pose.position.x - drop1Pose.position.x))
                .lineToY(sample2Pose.position.y)
                        .build();

        Action dropSample2 = drive.actionBuilder(sample2Pose)
                .setTangent(Math.atan2(drop2Pose.position.y - sample2Pose.position.y, drop2Pose.position.x - sample2Pose.position.x))
                .lineToY(drop2Pose.position.y)
                        .build();

        Action grabSample3 = drive.actionBuilder(drop2Pose)
                .setTangent(Math.atan2(sample3Pose.position.y - drop2Pose.position.y, sample3Pose.position.x - drop2Pose.position.x))
                .lineToYLinearHeading(sample3Pose.position.y, sample3Pose.heading)
                        .build();

        Action dropSample3 = drive.actionBuilder(sample3Pose)
                .setTangent(Math.atan2(drop3Pose.position.y - sample3Pose.position.y, drop3Pose.position.x - sample3Pose.position.x))
                .lineToXLinearHeading(drop3Pose.position.x, drop3Pose.heading)
                        .build();

        Action score2 = drive.actionBuilder(drop3Pose)
                .setTangent(Math.atan2(rungPose.position.y - drop3Pose.position.y, rungPose.position.x - drop3Pose.position.x))
                .lineToX(rungPose.position.x+1.5)
                .build();

        Action pickup2 = drive.actionBuilder(rungPose)
                .setTangent(Math.atan2(pickupPose.position.y - rungPose.position.y, pickupPose.position.x - rungPose.position.x))
                .lineToXLinearHeading(pickupPose.position.x, pickupPose.heading)
                        .build();

        Action score3 = drive.actionBuilder(drop3Pose)
                .setTangent(Math.atan2(rungPose.position.y - pickupPose.position.y, rungPose.position.x - pickupPose.position.x))
                .lineToX(rungPose.position.x+3)
                .build();

        robot.init(true);

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(  // go to score first specimen
                                score1,
                                elbow.elbowToDeg(112),
                                extension.extenderToInch(8.5),
                                axial.rotateAxial(robot.CLAW_UP)
                        ),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),

                        pinch.openClaw(),
                        new ParallelAction( // release specimen and retract arm/extender
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(37)
                        ),

                        grabSample1, // go to next sample and pick up
                        new ParallelAction(
                                elbow.elbowToDeg(0),
                                axial.rotateAxial(robot.CLAW_DOWN)
                        ),

                        sleepAction(500),
                        pinch.closeClaw(),
                        sleepAction(500),

                        new ParallelAction( // drop off sample
                                dropSample1,
                                elbow.elbowToDeg(180+37),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID)
                        ),
                        pinch.openClaw(),

                        new ParallelAction( // go to next sample
                                grabSample2,
                                elbow.elbowToDeg(0)
                        ),
                        axial.rotateAxial(robot.CLAW_DOWN),
                        sleepAction(500),
                        pinch.closeClaw(),
                        sleepAction(500),

                        new ParallelAction( // drop off 2nd sample
                                dropSample2,
                                elbow.elbowToDeg(180+37),
                                yaw.rotateClaw(robot.YAW_MID),
                                axial.rotateAxial(robot.CLAW_MID),
                                extension.extenderToInch(0)
                        ),
                        pinch.openClaw(),
                        new ParallelAction(
                                grabSample3,
                                elbow.elbowToDeg(37)
                        ),

                        new ParallelAction( // go to 3rd sample, rotate bot & claw
                                elbow.elbowToDeg(21),
                                yaw.rotateClaw(0.45),
                                extension.extenderToInch(8.25),
                                axial.rotateAxial(0.25)
                        ),
                        sleepAction(500),
                        pinch.closeClaw(),
                        sleepAction(500),

                        new ParallelAction( // retract and drop off 3rd sample
                                dropSample3,
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(180+37),
                                yaw.rotateClaw(robot.YAW_MID)
                        ),
                        pinch.openClaw(),

                        sleepAction(1000),

                        new ParallelAction( // pick up specimen FROM GROUND
                                elbow.elbowToDeg(180+37+16),
                                extension.extenderToInch(4.25),
                                axial.rotateAxial(robot.CLAW_MID + 0.15)
                        ),

                        sleepAction(1000),
                        pinch.closeClaw(),
                        extension.extenderToInch(0),

                        new ParallelAction( // drop off 2nd specimen
                                score2,
                                elbow.elbowToDeg(112),
                                extension.extenderToInch(8.5),
                                axial.rotateAxial(robot.CLAW_UP),
                                yaw.rotateClaw(robot.YAW_RIGHT)
                        ),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),

                        pinch.openClaw(),
                        new ParallelAction( // retract & come back for pickup FROM WALL
                                extension.extenderToInch(6),
                                elbow.elbowToDeg(180+37),
                                pickup2
                        ),
                        pinch.closeClaw(),
                        sleepAction(500),

                        new ParallelAction( // score 3rd specimen
                                score3,
                                elbow.elbowToDeg(112),
                                extension.extenderToInch(8.5),
                                axial.rotateAxial(robot.CLAW_UP),
                                yaw.rotateClaw(robot.YAW_MID)
                        ),
                        axial.rotateAxial(0.35),
                        elbow.elbowToDeg(60),

                        pinch.openClaw(), // come back & retract for parking
                        new ParallelAction(
                                extension.extenderToInch(0),
                                elbow.elbowToDeg(180+37),
                                pickup2
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
