package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@TeleOp(name = "BazaarTeleOp", group = "Robot")
public class BazaarTeleOp extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.colorDetector colorDetector = new VisionSoftware.colorDetector(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // initialization phase...
        double drive;
        double strafe;
        double turn;
        double elbowPos = 0; // just here to keep intelliJ quiet
        double elbowFactor;
        double extendFactor;

        boolean calibrateParallel = false;
        boolean calibratePerpendicular = false;
        boolean roboMode;

        boolean contorller1Mode = true; // true will be basic/default, false will be alternative (for both gamepads)
        boolean controller2Mode = true;

        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            if (gamepad1.a){
                contorller1Mode = !contorller1Mode;
            }

            if (contorller1Mode) {
                if (gamepad1.right_trigger > 0) { // slow down driving
                    double multiplier = -gamepad1.right_trigger + 1; // reverse trigger (it goes from 0 to 1, bad!)
                    drive = gamepad1.left_stick_y * multiplier;
                    strafe = (-gamepad1.left_stick_x * 1.1) * multiplier;
                    turn = gamepad1.right_stick_x * multiplier;
                }
                if (gamepad1.left_trigger > 0) { // release friction
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
            } else if (!contorller1Mode) {
                if (gamepad1.right_trigger > 0) {
                    extendFactor = gamepad1.right_trigger * robot.EXTENSION_FUDGE_FACTOR;
                    robot.extensionDrive.setTargetPosition((int) (robot.extensionDrive.getCurrentPosition() + extendFactor));
                } else if (gamepad1.right_trigger > 0) {
                    extendFactor = -gamepad1.left_trigger * robot.EXTENSION_FUDGE_FACTOR;
                    robot.extensionDrive.setTargetPosition((int) (robot.extensionDrive.getCurrentPosition() + extendFactor));
                }
            }
            robot.driveFieldCentric(drive,strafe,turn);


            // gamepad 2
            if (gamepad2.a && gamepad2.dpad_down) {
                controller2Mode = !controller2Mode;
            }

            if (gamepad2.left_stick_button) {
                calibrateParallel = !calibrateParallel;
                calibratePerpendicular = false;
            } else if (gamepad2.right_stick_button) {
                calibratePerpendicular = !calibratePerpendicular;
                calibrateParallel = false;
            }

            if (controller2Mode){

            } else if (!controller2Mode) {

            }
        }

    }
}
