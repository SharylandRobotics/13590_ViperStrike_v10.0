package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@TeleOp(name = "Field Centric", group = "Robot")
public class TeleOpDriveFieldCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot."
    // to access this class.
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    //VisionSoftware vision = new VisionSoftware(this);

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double elbowPos = 0; // just here to keep intelliJ quiet
        double elbowFactor;
        double extend;

        boolean calibrate = false; // Used to keep user changes for this variable
        boolean calibrateP = false;

        // Initialize all the hardware, using the hardware class.
        robot.init();

        // DO NOT MOVE ROBOT HERE: IT WILL BE A PENALTY!
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP).
        while (opModeIsActive()) {

            // Field Centric Mode use the left joystick to go forward & strafe,
            // and the right joystick to rotate from the perspective of the driver
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            turn = gamepad1.right_stick_x;

            // Find out how to use left_stick_y for extension movement FIXME
            extend = -gamepad2.left_stick_y;

            if (gamepad1.right_trigger != 0) { // slow down driving
                double multiplier = -gamepad1.right_trigger + 1; // reverse trigger (it goes from 0 to 1, bad!)
                drive = gamepad1.left_stick_y * multiplier;
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

            // Combine drive, strafe, and turn for blended motion. Use RobotHardware class
            robot.driveFieldCentric(drive, strafe, turn);

            // Close and Open Claw
            if (gamepad2.x) {
                robot.setClawPosition(robot.enable,robot.pass,robot.pass);
            } else if (gamepad2.b) {
                robot.setClawPosition(robot.disable,robot.pass,robot.pass);
            }

            // Raise and Lower Claw
            if (gamepad2.left_bumper) {
                robot.setClawPosition(robot.pass,robot.pass,robot.enable);
            } else if (gamepad2.right_bumper) {
                robot.setClawPosition(robot.pass,robot.pass,robot.disable);
            }

            // Rotate Claw here

            // Calibrate Claw Toggle
            if (gamepad2.right_stick_button) {
                calibrate = !calibrate;
                if (calibrate) {
                    calibrateP = false;
                }
            }
            if (gamepad2.left_stick_button) {
                calibrateP = !calibrateP;
                if (calibrateP) {
                    calibrate = false;
                }
            }


            // Drive Elbow
            // VERYY JUMPY, ESPECIALLY GOING AGAINST GRAVITY. consider using smt like GoBILDA example
            if (gamepad2.y) {
                elbowFactor = robot.ELBOW_FUDGE_FACTOR * -1;
                elbowPos = robot.elbowDrive.getCurrentPosition();
            } else if (gamepad2.a) {
                elbowFactor = robot.ELBOW_FUDGE_FACTOR * 1;
                elbowPos = robot.elbowDrive.getCurrentPosition();
            } else {
                elbowFactor = robot.ELBOW_FUDGE_FACTOR * 0;
            }

            // Extension Logic, don't move if at/past limit
            if (robot.extensionDrive.getCurrentPosition() >= robot.EXTENSION_MAXIMUM_COUNT) {
                robot.extensionDrive.setPower(0);
            } else {
                robot.extensionDrive.setPower(extend);
                // Drive Extension
                if (gamepad2.left_trigger != 0) {
                    robot.extensionDrive.setPower(1.0);
                } else if (gamepad2.right_trigger != 0) {
                    robot.extensionDrive.setPower(-1.0);
                } else {
                    robot.extensionDrive.setPower(0.0);
                }
            }

            // Set Elbow Positions
            if (gamepad2.dpad_up) {
                elbowPos = robot.ELBOW_PERPENDICULAR;
            } else if (gamepad2.dpad_right) {
                elbowPos = robot.ELBOW_BACKWARD_ANGLED;
            } else if (gamepad2.dpad_down) {
                elbowPos = robot.ELBOW_BACKWARD_COLLAPSED;
                robot.clawAxial.setPosition(robot.CLAW_UP);
            } else if (gamepad2.dpad_left) {
                elbowPos = robot.ELBOW_ANGLED;
            }/* else if (gamepad2.left_stick_button) {
                elbowPos = robot.ELBOW_ANTI_COLLAPSED;

            }
            */

            // Call setTargetPosition for elbow, ensure input is int
            robot.elbowDrive.setTargetPosition((int) (elbowPos + (int) elbowFactor));


            // Check to calibrate the claw
            if (calibrate) {
                robot.calibrateClaw(robot.ELBOW_PARALLEL);
            } else if (calibrateP) {
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }

            // Send a telemetry message to explain controls and show robot status
            telemetry.addData("Status", "Run Time: " + runtime.seconds());
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData("GAMEPAD2 LEFT STICK Y: ", String.valueOf(gamepad2.left_stick_y));
            telemetry.addData("ELBOW POSITION: ", String.valueOf(robot.elbowDrive.getCurrentPosition()));
            telemetry.addData("EXTENSION POSITION: ", String.valueOf(robot.extensionDrive.getCurrentPosition()));
            telemetry.update();
            // Place this loop so hands move at a reasonable speed
            sleep(50);
        }
    }
}
