package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "BazaarTeleOp", group = "Robot")
public class BazaarTeleOp extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    Limelight3A limelight;

    private boolean coloredFound;
    private boolean yellowFound;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        Pose3D botPose = null;
        // timers
        int leftBumperTimer = 0;
        int timesCounted = 0;
        int stickCounter = 0;
        int stickCounter2 = 0;
        // sounds
        int coloredSoundID = hardwareMap.appContext.getResources().getIdentifier("colored", "raw", hardwareMap.appContext.getPackageName());
        int yellowSoundID   = hardwareMap.appContext.getResources().getIdentifier("yellow",   "raw", hardwareMap.appContext.getPackageName());
        // preload sounds
        if (coloredSoundID != 0){ coloredFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, coloredSoundID); }
        if (yellowSoundID != 0){ yellowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, yellowSoundID); }
        telemetry.addData("gold resource",   coloredFound ?   "Found" : "NOT found\n Add colored.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", yellowFound ? "Found" : "Not found\n Add yellow.wav to /src/main/res/raw" );
        SoundPlayer.getInstance().setMasterVolume(2);
        // initialization phase...
        double drive;
        double strafe;
        double turn;

        double elbowFactor;
        double rotateFactor;

        boolean elbowByExtender = false;
        boolean g2RightStickSwitch = false;
        boolean calibratePerpendicular = false;

        double prevElbowPos;

        robot.init(false);
        double NEWelbowPos = robot.elbowDrive.getCurrentPosition();

        // extension encoder setup
        robot.extensionDrive.setTargetPosition(0);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(1.0);

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            prevElbowPos = robot.elbowDrive.getCurrentPosition();

            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = -gamepad1.right_stick_x;

            // RSB for keeping the servo where u had it
            if (gamepad2.right_stick_button && stickCounter2 >= 3){
                g2RightStickSwitch = !g2RightStickSwitch;
                stickCounter2 = 0;
            }
            if (g2RightStickSwitch){
                rotateFactor = robot.clawYaw.getPosition();
            } else {
                rotateFactor = (gamepad2.right_stick_x*-0.5) + 0.5;
            }

            // get LL results
            if (limelight.getLatestResult() != null) {
                botPose = limelight.getLatestResult().getBotpose();
            }

            // what it says
            if (gamepad1.a) { 
                if (botPose != null) {
                    NEWelbowPos = (robot.elbowTrigPosition(botPose, robot.heading));
                }
            }

            if (gamepad1.right_trigger != 0) { // slow down driving
                double multiplier = -gamepad1.right_trigger + 1; // reverse trigger (it goes from 0 to 1, bad!)
                drive = -gamepad1.left_stick_y * multiplier;
                strafe = (gamepad1.left_stick_x * 1.1) * multiplier;
                turn = gamepad1.right_stick_x * multiplier;
            }
            if (gamepad1.left_trigger != 0) { // release friction
                robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            robot.driveFieldCentric(drive, strafe, turn);


            // gamepad 2

            // toggle claw mode
             if (gamepad2.left_stick_button && stickCounter > 5) { // set claw perpendicular to floor
                calibratePerpendicular = !calibratePerpendicular;
                stickCounter = 0;
            }

             // claw positions
            if (gamepad2.x) { // close claw
                robot.clawPinch.setPosition(robot.CLAW_CLOSE);
            } else if (gamepad2.b) { // open claw
                robot.clawPinch.setPosition(robot.CLAW_OPEN);
            }
            if (gamepad2.a) { // down claw
                robot.clawAxial.setPosition(robot.CLAW_DOWN);
                calibratePerpendicular = false;
            } else if (gamepad2.y) { // claw mid
                robot.clawAxial.setPosition(robot.CLAW_MID);
            }

            // elbow pre-set positions
            if (gamepad2.dpad_up) {
                NEWelbowPos = ((int) robot.ELBOW_PERPENDICULAR);
            } else if (gamepad2.dpad_right) {
                NEWelbowPos = (int) robot.ELBOW_PARALLEL;
            } else if (gamepad2.dpad_left) {
                NEWelbowPos = ((int) robot.ELBOW_BACKWARD_PARALLEL);
            } else if (gamepad2.dpad_down) { // slightly off ground
                NEWelbowPos = (int) (robot.ELBOW_COLLAPSED + robot.angleConvert(10));
            }

            // shortcut to not preoccupy driver
            if (gamepad2.right_bumper) {
                robot.extensionDrive.setTargetPosition((int) robot.EXTENSION_MAXIMUM_COUNT);
            }

            // drive elbow manually
            // check if triggers are active...
            if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                if (gamepad2.left_trigger > 0.5) {
                    elbowFactor = Math.pow(5 * gamepad2.left_trigger, 3) * -robot.ELBOW_FUDGE_FACTOR;
                } else if (gamepad2.right_trigger > 0.5) {
                    elbowFactor = Math.pow(5 * gamepad2.right_trigger, 3) * robot.ELBOW_FUDGE_FACTOR;
                } else {
                    elbowFactor = robot.ELBOW_FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
                }
                // if they are drive and change the target position
                NEWelbowPos = robot.elbowDrive.getCurrentPosition() + (int) elbowFactor;

            }

            // drive elbow
            robot.elbowDrive.setTargetPosition((int) NEWelbowPos);
            // drive extension
            // happy, reader?
            robot.driveExtenderPosition(-gamepad2.left_stick_y);

            // drive heading servo
            robot.clawYaw.setPosition(rotateFactor);

            // actually calibrate the claw
            if (calibratePerpendicular) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }

            // drive arm by extender
            if (gamepad2.left_bumper) {
                if (leftBumperTimer > 10) {
                    elbowByExtender = !elbowByExtender;
                    leftBumperTimer = 0;
                }
                // if changed to false, start retracting the extender to near 0 (it can be intercepted, it's not a forced move!!)
                if (!elbowByExtender){robot.extensionDrive.setTargetPosition((int) (0.1*robot.EXTENSION_COUNTS_PER_REV));}
            } else if (elbowByExtender) {robot.elbowDrive.setTargetPosition((int) robot.armByExtender());}

            // telemetry
            telemetry.addData("Heading", robot.heading);
            telemetry.addData("Manual Driving", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("Elbow Position", robot.elbowDrive.getCurrentPosition() / robot.ARM_COUNTS_PER_DEGREE);
            telemetry.addData("Extender Position", robot.extensionDrive.getCurrentPosition() / robot.EXTENSION_COUNTS_PER_REV);
            telemetry.addData("Claw Calibration", "Perpendicular?", calibratePerpendicular);
            telemetry.addData("Elbow Mode:", elbowByExtender ? "Extender Based" : "Free Range");
            telemetry.update();
            switch ((int) runtime.seconds()) {
                case 90:
                    gamepad1.rumble(0.1, 0.1, 200);
                    break;
                case 120:
                case 135:
                    gamepad1.rumble(0.5, 0.5, 200);
                    break;
            }
            // check for elbow stalls
            if (prevElbowPos == robot.elbowDrive.getCurrentPosition() && robot.elbowDrive.getPower() >= 1.0){
                if (Math.abs(robot.elbowDrive.getCurrentPosition() - robot.elbowDrive.getTargetPosition()) > 4) {
                    timesCounted++;
                    if (timesCounted >= 10) {
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, coloredSoundID);
                    }
                }
            }
            stickCounter++;
            leftBumperTimer++;
            stickCounter2++;
            sleep(50);
        }
    }
}
