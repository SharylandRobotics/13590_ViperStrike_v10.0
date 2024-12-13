package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Testing :(", group = "Robot")

public class Tester extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double extend;
        double elbowFactor = 0;
        boolean calibrate = false;

        YawPitchRollAngles yawPitchRoll;
        double heading;
        // Initialize all the hardware using the hardware class.
        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);

        robot.extensionDrive.setTargetPosition(0);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(1.0);

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive() ) {
             // FIXME
            yawPitchRoll = robot.imu.getRobotYawPitchRollAngles(); // set orientation
            heading = yawPitchRoll.getYaw(); // set Yaw angle
            telemetry.addData("current orientation", String.valueOf(yawPitchRoll));
            telemetry.addData("current yaw", String.valueOf(heading));
            telemetry.addData("Elbow Angle:", robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE);
            telemetry.addData("Extension Rev:", robot.extensionDrive.getCurrentPosition()/robot.EXTENSION_COUNTS_PER_REV);
            telemetry.addData("Extension Pos:", robot.extensionDrive.getCurrentPosition());
            telemetry.addData("Elbow Pos:", robot.elbowDrive.getCurrentPosition());
            telemetry.update();
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            turn = gamepad1.right_stick_x;
            robot.driveFieldCentric(drive, strafe,turn);

            if (gamepad2.left_stick_button) {
                calibrate = !calibrate;
            }
            if (calibrate){robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);}



            extend = -gamepad2.left_stick_y * robot.EXTENSION_FUDGE_FACTOR;
            if (robot.extensionDrive.getCurrentPosition() >= robot.EXTENSION_MAXIMUM_COUNT){extend = 0;}
            robot.extensionDrive.setTargetPosition(robot.extensionDrive.getCurrentPosition() + (int) extend);

            if (gamepad2.left_trigger != 0) {
                elbowFactor = gamepad2.left_trigger * -robot.ELBOW_FUDGE_FACTOR;
            } else if (gamepad2.right_trigger != 0) {
                elbowFactor = gamepad2.right_trigger * robot.ELBOW_FUDGE_FACTOR;
            } else { elbowFactor = 0;}
            robot.elbowDrive.setTargetPosition(robot.elbowDrive.getCurrentPosition() + (int) elbowFactor);
            if (gamepad2.dpad_left) {robot.extensionDrive.setTargetPosition((int) robot.EXTENSION_COUNTS_PER_REV*12);}
            sleep(50);
        }
    }
}
