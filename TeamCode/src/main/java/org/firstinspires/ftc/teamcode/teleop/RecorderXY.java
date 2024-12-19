package org.firstinspires.ftc.teamcode.teleop;

import java.io.FileWriter;
import java.io.IOException;

//https://www.youtube.com/watch?v=OFX96ONRU18

import android.os.Environment;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "RecorderXY/debug", group = "Robot")
public class RecorderXY extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    ElapsedTime runtime = new ElapsedTime();
    String logFilePath = String.format("%s/FIRST/xyrecordlog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    FileWriter writer = new FileWriter(logFilePath);

    public RecorderXY() throws IOException {
    }

    public void record(double x, double y, double heading, double armPos, double clawPos, double axialPos, boolean APTempty, AprilTagDetection APTobject){
        try {
            writer.write(x + ", " + y + ", " + heading + ", " + armPos + ", " + clawPos + ", " + axialPos + ", " + (APTempty ? "-1, " : APTobject.ftcPose.bearing + ", "));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void runOpMode() {
        boolean coloredFound =false;
        boolean yellowFound = false;
        boolean notificationFound = false;

        double drive;
        double strafe;
        double turn;

        double elbowFactor;
        boolean calibrateParallel = false;
        boolean calibratePerpendicular = false;

        double clawPos = 0.4;
        double axialPos = 0.75;
        double armPos;
        double heading = 0;

        int actionCounter = 0;
        int burstCounter = 0;
        int permaCounter = 0;

        AprilTagDetection APTobject;
        double robotPosX = 0;
        double robotPosY = 0;
        boolean APTempty;

        boolean burstMode = false;
        boolean permaMode = false;

        int coloredSoundID = hardwareMap.appContext.getResources().getIdentifier("colored", "raw", hardwareMap.appContext.getPackageName());
        int yellowSoundID   = hardwareMap.appContext.getResources().getIdentifier("yellow",   "raw", hardwareMap.appContext.getPackageName());
        int notificationID = hardwareMap.appContext.getResources().getIdentifier("notification", "raw", hardwareMap.appContext.getPackageName());
        // preload sounds
        if (notificationID != 0){ notificationFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, notificationID);}
        if (coloredSoundID != 0){ coloredFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, coloredSoundID); }
        if (yellowSoundID != 0){ yellowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, yellowSoundID); }
        telemetry.addData("color resource",   coloredFound ?   "Found" : "NOT found\n Add colored.wav to /src/main/res/raw" );
        telemetry.addData("yellow resource", yellowFound ? "Found" : "Not found\n Add yellow.wav to /src/main/res/raw" );
        telemetry.addData("notification resource", notificationFound ? "Found" : "Not found\n Add notification.wav to /src/main/res/raw" );
        SoundPlayer.getInstance().setMasterVolume(2);

        aptDetector.visionInit();
        robot.init();
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);

        while (opModeInInit()) {
            robot.clawPinch.setPosition(0.4);
            // close claw
            robot.clawAxial.setPosition(0.75);
            // set claw to middle
        }

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            heading = robot.heading;

            robot.driveFieldCentric(drive, strafe, turn);

            aptDetector.activeAPTscanner(-1);
            APTobject = aptDetector.detectedTag;
            if (APTobject != null) {
                robotPosX = APTobject.robotPose.getPosition().x;
                robotPosY = APTobject.robotPose.getPosition().y;
                APTempty = false;
            } else { APTempty = true; }


            if (gamepad1.left_trigger != 0) {
                elbowFactor = gamepad1.left_trigger * -robot.ELBOW_FUDGE_FACTOR;
            } else if (gamepad1.right_trigger != 0) {
                elbowFactor = gamepad1.right_trigger * robot.ELBOW_FUDGE_FACTOR;
            } else { elbowFactor = 0;}
            robot.elbowDrive.setTargetPosition(robot.elbowDrive.getCurrentPosition() + (int) elbowFactor);
            armPos = robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE;

            if (gamepad1.x) { // close claw
                robot.setClawPosition(robot.enable, robot.pass, robot.pass);
                clawPos = 0.4;
            } else if (gamepad1.b) { // open claw
                robot.setClawPosition(robot.disable, robot.pass, robot.pass);
                clawPos = 0.1;
            }

            if (gamepad1.dpad_down) {
                calibrateParallel = !calibrateParallel; calibratePerpendicular = false;
            } else if (gamepad1.dpad_right) {
                calibratePerpendicular = !calibratePerpendicular; calibrateParallel = false;
            } else if (gamepad1.a) {
                calibrateParallel = false; calibratePerpendicular = false; robot.clawAxial.setPosition(0.5);
            }

            if (calibrateParallel) { // actually calibrate the claw
                robot.calibrateClaw(robot.ELBOW_PARALLEL);
            } else if (calibratePerpendicular) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            axialPos = robot.clawAxial.getPosition();
            robot.elbowDrive.setTargetPosition(robot.elbowDrive.getCurrentPosition() + (int) elbowFactor);


            telemetry.addData("Action: ", actionCounter);
            telemetry.update();

            if (gamepad2.right_bumper) {
                burstMode = !burstMode;
                permaMode = false;
            } else if (gamepad2.dpad_down) {
                permaMode = !permaMode;
                burstMode = false;
            }

            if (burstMode) {
                permaMode = false;
                burstCounter++;
                if (burstCounter == 400){
                    record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                    actionCounter++;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, notificationID);
                } else if (burstCounter == 600) {
                    record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                    actionCounter++;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, notificationID);
                } else if (burstCounter == 800) {
                    record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                    actionCounter++;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, notificationID);
                } else if (burstCounter == 1000) {
                    record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                    actionCounter++;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, notificationID);
                } else if (burstCounter == 1200) {
                    record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                    actionCounter++;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, notificationID);
                }else {
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, yellowSoundID);
                }
            } else if (permaMode) {
                record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                actionCounter++;
                burstMode = false;
            } else if (gamepad2.left_bumper) {
                record(robotPosX, robotPosY, heading, armPos, clawPos, axialPos, APTempty, APTobject);
                actionCounter++;
                burstMode = false;
                permaMode = false;
            }



            sleep(50);
        }

        try {
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}