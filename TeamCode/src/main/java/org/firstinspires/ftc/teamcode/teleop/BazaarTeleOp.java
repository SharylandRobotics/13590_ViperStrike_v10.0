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
        // timers
        int leftBumperTimer = 0;
        int gpad2Timer = 0;
        int timesRan = 0;
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

        double elbowFactor;
        double extendFactor;
        double rotateFactor;

        boolean elbowByExtender = false;
        boolean calibrateParallel = false;
        boolean calibratePerpendicular = false;
        boolean cameraMode = false;

        // true will be basic/default, false will be alternative (for both gamepads)
        boolean controller2Mode = true;

        robot.visionInit("BLUE", true, -0.6,0.6,0.6,-0.6);
        robot.init();
        double NEWelbowPos = robot.elbowDrive.getCurrentPosition(); // just here to keep intelliJ quiet

        // extension encoder setup
        robot.extensionDrive.setTargetPosition(0);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            extendFactor = -gamepad2.left_stick_y * robot.EXTENSION_FUDGE_FACTOR;
            rotateFactor = (-gamepad2.right_stick_x*-0.5) + 0.5;
            if (gamepad2.right_stick_y > 0.8){rotateFactor = 0.5;} // move stick up to reset to middle
            // or if you want it incrementally : (-gamepad2.right_stick_x*-0.5) + 0.5) * 0.01 *adjust for sensitivity*
            // also change the set servo pos to getPosition() + rotateFactor

            while (gamepad1.dpad_left && gamepad1.dpad_right) { // reset init
                timesRan = timesRan + 1;
                if (timesRan >= 200) {
                    robot.init();
                    timesRan = 0;
                    break;
                }
                sleep(50);
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
            if (gamepad2.right_bumper) { // swap modes
                if (gpad2Timer > 20) {
                    controller2Mode = !controller2Mode;
                    gpad2Timer = 0;
                }
            }

            if (gamepad2.left_bumper) {
                if (leftBumperTimer > 10) {
                    elbowByExtender = !elbowByExtender;
                    leftBumperTimer = 0;

                }
                // if changed to false, start retracting the extender to near 0 (it can be intercepted, it's not a forced move!!)
                if (!elbowByExtender){robot.extensionDrive.setTargetPosition((int) (0.1*robot.EXTENSION_COUNTS_PER_REV));}
            } else if (elbowByExtender) {NEWelbowPos = robot.armByExtender();}


            if (gamepad2.right_stick_button) { // set claw parallel to floor
                calibrateParallel = !calibrateParallel;
                calibratePerpendicular = false;
            } else if (gamepad2.left_stick_button) { // set claw perpendicular to floor
                calibratePerpendicular = !calibratePerpendicular;
                calibrateParallel = false;
            }

            if (gamepad2.x) { // close claw
                robot.setClawPosition(robot.enable, robot.pass, robot.pass);
            } else if (gamepad2.b) { // open claw
                robot.setClawPosition(robot.disable, robot.pass, robot.pass);
            }
            if (gamepad2.a) { // mid claw
                robot.clawAxial.setPosition(robot.CLAW_MID);
            }

            if (gamepad2.dpad_up) {
                robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PERPENDICULAR);
            }

            if (gamepad2.left_trigger != 0) {
                extendFactor = gamepad2.left_trigger * -robot.EXTENSION_FUDGE_FACTOR;
            } else if (gamepad2.right_trigger != 0) {
                extendFactor = gamepad2.left_trigger * robot.EXTENSION_FUDGE_FACTOR;
            }

            if (controller2Mode) { // controller mode default
                cameraMode = false;

                if (gamepad2.dpad_right) {
                    NEWelbowPos = ((int) robot.ELBOW_PARALLEL);
                } else if (gamepad2.dpad_left) {
                    NEWelbowPos = ((int) robot.ELBOW_BACKWARD_PARALLEL);
                } else if (gamepad2.dpad_down) {
                    NEWelbowPos = ((int) robot.ELBOW_COLLAPSED);
                } else {
                    NEWelbowPos = robot.elbowDrive.getCurrentPosition();
                }
            }
            if (!controller2Mode) { // controller 2 mode alternative
                if (gamepad2.dpad_down) {
                    cameraMode = !cameraMode;
                }
                if (gamepad2.dpad_left) { // move elbow down
                    NEWelbowPos = (robot.elbowDrive.getCurrentPosition() - (int) robot.angleConvert(2));
                } else if (gamepad2.dpad_right) { // move elbow up
                    NEWelbowPos = (robot.elbowDrive.getCurrentPosition() + (int) robot.angleConvert(2));
                } else {
                    NEWelbowPos = robot.elbowDrive.getCurrentPosition();// keep from creating a loop
                }
            }

            // MISC/ACTION code

            // drive extension
            // ensure you don't hit the extender limit!!
            if (robot.extensionDrive.getCurrentPosition() >= (int) robot.EXTENSION_MAXIMUM_COUNT && extendFactor >= 0){extendFactor = 0;}
            else if (robot.extensionDrive.getCurrentPosition() <= (int) (0.1*robot.EXTENSION_COUNTS_PER_REV) && extendFactor <= 0) {extendFactor = 0;}
            robot.extensionDrive.setTargetPosition(robot.extensionDrive.getCurrentPosition() + (int) extendFactor);
            // drive arm
            robot.elbowDrive.setTargetPosition((int) NEWelbowPos);
            // drive heading servo
            robot.clawYaw.setPosition(rotateFactor);

            if (calibrateParallel) { // actually calibrate the claw
                robot.calibrateClaw(robot.ELBOW_PARALLEL);
            } else if (calibratePerpendicular) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            if (cameraMode) { // play sounds if colors found
                robot.teleOpdetectR(new Point(480, 810), new Point(1440, 270));
                if (!robot.blobS.isEmpty()) {
                    SoundPlayer.getInstance().stopPlayingAll();
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, coloredSoundID);
                }
                if (!robot.secondaryBlobS.isEmpty()) {
                    SoundPlayer.getInstance().stopPlayingAll();
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, yellowSoundID);
                }
            }

            // telemetry
            telemetry.addData("Heading", robot.heading);
            telemetry.addData("Manual Driving", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("Elbow Position", robot.elbowDrive.getCurrentPosition() / robot.COUNTS_PER_DEGREE);
            telemetry.addData("Extender Position", robot.extensionDrive.getCurrentPosition() / robot.EXTENSION_COUNTS_PER_REV);
            telemetry.addData("Claw Calibration", "Parallel, Perpendicular", calibrateParallel, calibratePerpendicular);
            telemetry.addData("GamePad2 Mode:", controller2Mode ? "Default/Positional" : "Alternative/Incremental");
            telemetry.addData("Elbow Mode:", elbowByExtender ? "Extender Based" : "Free Range");
            telemetry.update();
            switch ((int) runtime.seconds()) {
                case 90:
                    gamepad2.rumble(0.1, 0.1, 200);
                    break;
                case 120:
                    gamepad2.rumble(0.5, 0.5, 200);
                case 135:
                    gamepad2.rumble(0.5, 0.5, 200);
                }
            gpad2Timer++;
            leftBumperTimer++;
            sleep(50);
        }
    }
}
