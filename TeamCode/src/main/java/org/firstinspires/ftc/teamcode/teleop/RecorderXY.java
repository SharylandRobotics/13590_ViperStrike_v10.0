package org.firstinspires.ftc.teamcode.teleop;

import java.io.FileWriter;
import java.io.IOException;

//https://www.youtube.com/watch?v=OFX96ONRU18

import android.os.Environment;
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
    String logFilePath = String.format("%s/FIRST/recordlog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    FileWriter writer = new FileWriter(logFilePath);

    public RecorderXY() throws IOException {
    }


    @Override
    public void runOpMode() {
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

        AprilTagDetection APTobject;
        double robotPosX = 0;
        double robotPosY = 0;
        boolean APTempty;

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
            } else if (gamepad1.dpad_right) { calibratePerpendicular = !calibratePerpendicular; calibrateParallel = false;}

            if (calibrateParallel) { // actually calibrate the claw
                robot.calibrateClaw(robot.ELBOW_PARALLEL);
            } else if (calibratePerpendicular) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            axialPos = robot.clawAxial.getPosition();
            robot.elbowDrive.setTargetPosition(robot.elbowDrive.getCurrentPosition() + (int) elbowFactor);


            telemetry.addData("Action: ", actionCounter);
            telemetry.update();

            try {
                writer.write( robotPosX+", " + robotPosY +", " + heading+", " + armPos+", " + clawPos+", "+ axialPos+", " + (APTempty ? "-1, " : APTobject.ftcPose.bearing+ ", "));
                actionCounter++;
            } catch (IOException e) {
                throw new RuntimeException(e);
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