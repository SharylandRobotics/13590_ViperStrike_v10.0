package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Field Centric", group = "Robot")

public class TeleOpDriveFieldCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware function with "robot."
    // to access this class.
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double extend;

        // Initialize all the hardware, using the hardware class.
        robot.init();

        // DO NOT MOVE ROBOT HERE: IT WILL BE A PENALTY!

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
            //extend = gamepad2.left_stick_y; use for extension motor

            // Combine drive, strafe, and turn for blended motion. Use RobotHardware class
            robot.driveFieldCentric(drive, strafe, turn);



            if (gamepad2.x) { // Close and Open Claw
                robot.setClawPosition(robot.enable,0,robot.pass,0);
            } else if (gamepad2.b) {
                robot.setClawPosition(robot.disable,0,robot.pass,0);
            }


            if (gamepad2.left_bumper) { // Raise and Lower Claw
                robot.setClawPosition(robot.pass,0,robot.enable,0);
            } else if (gamepad2.right_bumper) {
                robot.setClawPosition(robot.pass,0,robot.disable,0);
            }


            if (gamepad2.left_trigger >= 0) { // Rotate Claw
                robot.setClawPosition(robot.pass,-gamepad2.left_trigger,robot.pass,0);
            } else if (gamepad2.right_trigger >= 0) {
                robot.setClawPosition(robot.pass,gamepad2.right_trigger,robot.pass,0);
            }

            if (gamepad2.y) { // Drive Lift
                robot.elbowDrive.setPower(1.0);
            } else if (gamepad2.a) {
                robot.elbowDrive.setPower(-1.0);
            }

            /* Use for Linear Actuator for arm extension, Monday
                robot.extensionDrive.setPower(gamepad2.left_stick_y);
             */

            // Send a telemetry message to explain controls and show robot status
            telemetry.addData("Status", "Run Time: " + runtime.seconds());
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            /*telemetry.addData("Servos&Motors", "Pinch: %.2f, Axial: %.2f, Lift: %5.2 ",
                    robot.clawPinch.getPosition(), robot.clawAxial.getPosition(),
                    robot.liftDrive.getCurrentPosition());
            telemetry.update();

             */

            // Place this loop so hands move at a reasonable speed
            sleep(50);
        }
    }
}
