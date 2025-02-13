package org.firstinspires.ftc.teamcode.roadRunnerAutonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RRactions;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;

@Config
@Autonomous(name = "RR Specimen Test", group = "RoadRunner")
public class RRtype1Auto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    RRactions actionLib = new RRactions(robot);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -61.25, Math.toRadians(90));
        Pose2d testPose = new Pose2d(9.5, -40, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        RRactions.Elbow elbow = actionLib.new Elbow(hardwareMap);
        RRactions.Extender extension = actionLib.new Extender(hardwareMap);
        RRactions.Pincher pinch = actionLib.new Pincher(hardwareMap);
        RRactions.Yaw yaw = actionLib.new Yaw(hardwareMap);
        RRactions.Axial axial = actionLib.new Axial(hardwareMap);

        Action leg1 = drive.actionBuilder(initialPose)
                .lineToY(-40)
                .waitSeconds(1)
                .build();

        Action leg2 = drive.actionBuilder(initialPose)
                .lineToXLinearHeading(35, Math.toRadians(45))
                .lineToY(-35)
                .build();

        Actions.runBlocking(pinch.closeClaw());
        Actions.runBlocking(axial.rotateAxial(robot.CLAW_MID));
        Actions.runBlocking(extension.extenderToInch(0));
        Actions.runBlocking(elbow.elbowToDeg(0));

        waitForStart();

        if (isStopRequested()) return ;

        Actions.runBlocking(
                new SequentialAction(
                    elbow.elbowToDeg(112),
                    leg1,
                    elbow.elbowToDeg(52),
                    pinch.openClaw(),
                    elbow.elbowToDeg(0),

                    leg2
                    // leg 2 actions

                )
        );
    }
}
