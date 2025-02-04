package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "SoloCameraOp", group = "Robot")
@Disabled
public class soloCam extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();



    private boolean coloredFound;
    private boolean yellowFound;
    @Override
    public void runOpMode() {
        // timers
        int leftBumperTimer = 0;
        int timesRan = 0;
        int timesCounted = 0;
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

        double extendFactor;

        boolean elbowByExtender = false;
        boolean calibrateParallel = false;
        boolean calibratePerpendicular = false;

        double prevElbowPos;

        robot.init(false);

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
            turn = gamepad1.right_stick_x;

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

            if (gamepad1.x) { // close claw
                robot.setClawPosition(robot.enable, robot.pass, robot.pass);
            } else if (gamepad1.b) { // open claw
                robot.setClawPosition(robot.disable, robot.pass, robot.pass);
            }

            // drive extension
            // ensure you don't hit the extender limit!! FIXME
            extendFactor = -gamepad1.left_stick_y * robot.EXTENSION_FUDGE_FACTOR;
            if ((robot.extensionDrive.getCurrentPosition() >= robot.EXTENSION_MAXIMUM_COUNT) && extendFactor >= 0) {extendFactor = 0;}
            if ((robot.extensionDrive.getCurrentPosition() <= 0) && extendFactor <= 0) {extendFactor = 0;}
            robot.extensionDrive.setTargetPosition(robot.extensionDrive.getCurrentPosition() + (int) extendFactor);
            // drive arm
            if (gamepad1.left_bumper) {
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
            telemetry.addData("Claw Calibration", "Parallel, Perpendicular", calibrateParallel, calibratePerpendicular);
            telemetry.addData("Elbow Mode:", elbowByExtender ? "Extender Based" : "Free Range");
            telemetry.update();

            // check for elbow stalls
            if (prevElbowPos == robot.elbowDrive.getCurrentPosition() && robot.elbowDrive.getPower() >= 1.0){
                if (Math.abs(robot.elbowDrive.getCurrentPosition() - robot.elbowDrive.getTargetPosition()) > 4) {
                    timesCounted++;
                    if (timesCounted >= 10) {
                        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, coloredSoundID);
                    }
                }
            }

            leftBumperTimer++;
            sleep(50);
        }
    }
}
