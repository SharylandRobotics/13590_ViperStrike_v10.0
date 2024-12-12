package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;
import org.opencv.core.Point;

@TeleOp(name = "BazaarTeleOp", group = "Robot")
public class BazaarTeleOp extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.colorDetector colorDetector = new VisionSoftware.colorDetector(this);
    ElapsedTime runtime = new ElapsedTime();



    private boolean coloredFound;
    private boolean yellowFound;
    @Override
    public void runOpMode() {
        // sounds
        int coloredSoundID = hardwareMap.appContext.getResources().getIdentifier("colored", "raw", hardwareMap.appContext.getPackageName());
        int yellowSoundID   = hardwareMap.appContext.getResources().getIdentifier("yellow",   "raw", hardwareMap.appContext.getPackageName());
        // preload sounds
        if (coloredSoundID != 0){ coloredFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, coloredSoundID); }
        if (coloredSoundID != 0){ yellowFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, yellowSoundID); }
        telemetry.addData("gold resource",   coloredFound ?   "Found" : "NOT found\n Add colored.wav to /src/main/res/raw" );
        telemetry.addData("silver resource", yellowFound ? "Found" : "Not found\n Add yellow.wav to /src/main/res/raw" );
        SoundPlayer.getInstance().setMasterVolume(2);
        // initialization phase...
        double drive;
        double strafe;
        double turn;
        double elbowPos = 0; // just here to keep intelliJ quiet
        double elbowFactor;
        double extendFactor;

        boolean calibrateParallel = false;
        boolean calibratePerpendicular = false;
        boolean cameraMode = false;

        boolean contorller1Mode = true; // true will be basic/default, false will be alternative (for both gamepads)
        boolean controller2Mode = true;

        robot.visionInit("BLUE", true, -0.6,0.6,0.6,-0.6);
        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            if (gamepad1.a){
                contorller1Mode = !contorller1Mode;
                telemetry.addData("MODE SWITCHED","");
            }

            if (contorller1Mode) {
                if (gamepad1.right_trigger != 0) { // slow down driving
                    double multiplier = -gamepad1.right_trigger + 1; // reverse trigger (it goes from 0 to 1, bad!)
                    drive = gamepad1.left_stick_y * multiplier;
                    strafe = (-gamepad1.left_stick_x * 1.1) * multiplier;
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
            }
            if (!contorller1Mode) {
                if (gamepad1.right_trigger != 0) {
                    extendFactor = gamepad1.right_trigger * robot.EXTENSION_FUDGE_FACTOR;
                    robot.extensionDrive.setTargetPosition((int) (robot.extensionDrive.getCurrentPosition() + extendFactor));
                } else if (gamepad1.right_trigger != 0) {
                    extendFactor = -gamepad1.left_trigger * robot.EXTENSION_FUDGE_FACTOR;
                    robot.extensionDrive.setTargetPosition((int) (robot.extensionDrive.getCurrentPosition() + extendFactor));
                }
            }
            robot.driveFieldCentric(drive,strafe,turn);


            // gamepad 2
            if (gamepad2.a && gamepad2.dpad_down) {
                controller2Mode = !controller2Mode;
            }

            if (gamepad2.left_stick_button) { // set claw parallel to floor
                calibrateParallel = !calibrateParallel;
                calibratePerpendicular = false;
            } else if (gamepad2.right_stick_button) { // set claw perpendicular to floor
                calibratePerpendicular = !calibratePerpendicular;
                calibrateParallel = false;
            }

            if (gamepad2.x) { // close claw
                robot.setClawPosition(robot.enable, robot.pass, robot.pass);
            } else if (gamepad2.b) { // open claw
                robot.setClawPosition(robot.disable, robot.pass, robot.pass);
            }

            if (gamepad2.dpad_up) {
                robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PERPENDICULAR);
            }

            if (controller2Mode){ // controller mode default
                cameraMode = false;

                if (gamepad2.dpad_right) {
                    robot.elbowDrive.setTargetPosition((int)  robot.ELBOW_PARALLEL);
                } else if (gamepad2.dpad_left) {
                    robot.elbowDrive.setTargetPosition((int) robot.ELBOW_BACKWARD_PARALLEL);
                } else if (gamepad2.dpad_down) {
                    robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_COLLAPSED + robot.angleConvert(30)));
                }
            }
            if (!controller2Mode) { // controller mode alternative
                if (gamepad2.dpad_down) {
                    cameraMode = !cameraMode;
                } else if (gamepad2.dpad_left) {
                    robot.elbowDrive.setTargetPosition(robot.elbowDrive.getCurrentPosition() - (int) robot.angleConvert(2));
                } else if (gamepad2.dpad_right) {
                    robot.elbowDrive.setTargetPosition(robot.elbowDrive.getCurrentPosition() + (int) robot.angleConvert(2));
                }
            }

            if (calibrateParallel) { // actually calibrate the claw
                robot.calibrateClaw(robot.ELBOW_PARALLEL);
            } else if (calibratePerpendicular) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            if (cameraMode) { // play sounds if color founds
                robot.teleOpdetectR(new Point(480,810), new Point(1440,270));
                if (!robot.blobS.isEmpty()){
                    SoundPlayer.getInstance().stopPlayingAll();
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, coloredSoundID);
                }
                if (!robot.secondaryBlobS.isEmpty()){
                    SoundPlayer.getInstance().stopPlayingAll();
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, yellowSoundID);
                }
            }

            // telemetry
            telemetry.addData("Heading", robot.heading);
            telemetry.addData("Manual Driving", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("Elbow Position", robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE);
            telemetry.addData("Extender Position", robot.extensionDrive.getCurrentPosition()/robot.EXTENSION_COUNTS_PER_REV);
            telemetry.addData("Claw Calibration", "Parallel, Perpendicular", calibrateParallel, calibratePerpendicular);
            telemetry.update();
        }



    }
}
