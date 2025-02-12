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
    private double NEWelbowPos;
    private double NEWextenderPos;
    private boolean calibratePerpendicular;

    private void checkDpad(){

    }

    private void checkLetters(){
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
            calibratePerpendicular = false;
        }
    }

    private void extensionHandler(){
        NEWextenderPos = robot.driveExtenderPosition(-gamepad2.left_stick_y);
        // shortcut to not preoccupy driver
        if (gamepad2.left_bumper) {
            robot.extensionDrive.setTargetPosition((int) robot.EXTENSION_MAXIMUM_COUNT);
        } else if ((int) NEWextenderPos != robot.extensionDrive.getCurrentPosition())
        { // check that the input given is actually doing smt as to not interrupt shortcut
            robot.extensionDrive.setTargetPosition((int) NEWextenderPos);
        }
    }

    private double elbowFactor(){
        double elbowFactor;
        if (gamepad2.left_trigger > 0.5) {
            elbowFactor = Math.pow(3 * gamepad2.left_trigger, 2) * -robot.ELBOW_FUDGE_FACTOR;
        } else if (gamepad2.right_trigger > 0.5) {
            elbowFactor = Math.pow(3 * gamepad2.right_trigger, 2) * robot.ELBOW_FUDGE_FACTOR;
        } else {
            elbowFactor = robot.ELBOW_FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
        }

        // avoid going past 0
        if (robot.elbowDrive.getCurrentPosition() + (int) elbowFactor < 0){
            elbowFactor -= (robot.elbowDrive.getCurrentPosition() + (int) elbowFactor);
        }

        return elbowFactor;
    }

    private void modeSwapper (DcMotor.RunMode mode){
        robot.leftFrontDrive.setMode(mode);
        robot.leftBackDrive.setMode(mode);
        robot.rightFrontDrive.setMode(mode);
        robot.rightBackDrive.setMode(mode);
    }

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

        double rotateFactor;

        boolean elbowByExtender = false;
        boolean g2RightStickSwitch = false;
        calibratePerpendicular = false;

        double prevElbowPos;
        double elbowFactor;

        robot.init(false);
        double NEWelbowPos = robot.elbowDrive.getCurrentPosition();
        double heading;

        // extension encoder setup
        robot.extensionDrive.setTargetPosition(0);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(1.0);

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        limelight.start();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
            prevElbowPos = robot.elbowDrive.getCurrentPosition();

            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = -gamepad1.right_stick_x;

            // RSB for keeping the servo where u had it
            if (gamepad2.right_stick_button && stickCounter2 >= 2){
                g2RightStickSwitch = !g2RightStickSwitch;
                stickCounter2 = 0;
            }
            if (g2RightStickSwitch){
                rotateFactor = robot.YAW_RIGHT;
            } else {
                rotateFactor = (gamepad2.right_stick_x*-0.5) + robot.YAW_MID;
            }

            // get LL results
            if (limelight.getLatestResult() != null) {
                botPose = limelight.getLatestResult().getBotpose();
            } else {
                telemetry.addData("not detecting", "");
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


            // rotate to face straight ahead ; you will lose control for a bit (no driving/strafing until its done)
            if (gamepad1.b) {
                double ortho;
                if (heading < 0){
                    ortho = -(180 + Math.round(heading));
                } else {
                    ortho = (180 - Math.round(heading));
                }
                robot.encoderFieldCentric(0,0,ortho);
                robot.driveFieldCentric(0,0, Math.abs(ortho)/ortho);
                modeSwapper(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // swap mode back when done with rotation
            if (!robot.leftFrontDrive.isBusy() || !robot.leftBackDrive.isBusy() || !robot.rightFrontDrive.isBusy() || !robot.rightBackDrive.isBusy()){
                modeSwapper(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.driveFieldCentric(drive, strafe, turn);
            }

            // gamepad 2

            // toggle claw mode
             if (gamepad2.left_stick_button && stickCounter > 5) { // set claw perpendicular to floor
                calibratePerpendicular = !calibratePerpendicular;
                stickCounter = 0;
            }

            checkLetters();
            // elbow pre-set positions DPAD
            if (gamepad2.dpad_up) {
                NEWelbowPos = ((int) robot.ELBOW_PERPENDICULAR);
            } else if (gamepad2.dpad_right) {
                NEWelbowPos = (int) (robot.angleConvert(33));
            } else if (gamepad2.dpad_left) {
                NEWelbowPos = ((int) robot.ELBOW_BACKWARD_PARALLEL);
            } else if (gamepad2.dpad_down) { // slightly off ground
                NEWelbowPos = (int) (robot.ELBOW_COLLAPSED);
            }

            // drive arm by extender: manual control has priority, movement here can be interrupted
            if (gamepad2.right_bumper) {
                if (leftBumperTimer > 7) {
                    elbowByExtender = !elbowByExtender;
                    leftBumperTimer = 0;
                }
                // if changed to false, start retracting the extender to near 0 (it can be intercepted, it's not a forced move!!)
                if (!elbowByExtender){
                    robot.extensionDrive.setTargetPosition(0);
                    NEWelbowPos = robot.ELBOW_PARALLEL;
                } else { // if switched on, move to preset pos : lowest elbow priority (can be intercepted)
                    NEWelbowPos = robot.ELBOW_COLLAPSED + robot.angleConvert(5);
                }
            }
            extensionHandler();

            // triggerHandler goes after bc manual control has priority
            if (elbowByExtender) {
                // only start moving elbow when you're in preset pos
                if (robot.elbowDrive.getCurrentPosition() < robot.ELBOW_PARALLEL - robot.angleConvert(10)) {
                    NEWelbowPos = (int) robot.armByExtender();
                }
            }

            if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                elbowFactor = elbowFactor();
                // if they are drive and change the target position
                NEWelbowPos = robot.elbowDrive.getCurrentPosition() + (int) elbowFactor;
            }




            // drive elbow
            robot.elbowDrive.setTargetPosition((int) NEWelbowPos);
            // drive heading servo
            robot.clawYaw.setPosition(rotateFactor);

            // actually calibrate the claw
            if (calibratePerpendicular) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }



            // telemetry
            telemetry.addData("Heading", heading);
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
