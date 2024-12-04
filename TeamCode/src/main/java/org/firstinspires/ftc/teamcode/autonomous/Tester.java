package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

        YawPitchRollAngles yawPitchRoll;
        double heading;
        // Initialize all the hardware using the hardware class.
        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(0.25);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PERPENDICULAR);

        /*
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_ANTI_COLLAPSED);
        robot.elbowDrive.setPower(0.3);
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawAxial.setPosition(0.3);

         */
        while (opModeIsActive() ) {
            robot.calibrateClaw(robot.ELBOW_PERPENDICULAR); // FIXME
            yawPitchRoll = robot.imu.getRobotYawPitchRollAngles(); // set orientation
            heading = yawPitchRoll.getYaw(); // set Yaw angle
            telemetry.addData("current orientation", String.valueOf(yawPitchRoll));
            telemetry.addData("current yaw", String.valueOf(heading));
            telemetry.addData("Elbow Pos:", robot.elbowDrive.getCurrentPosition());
            telemetry.addData("Elbow Angle:", Math.round(robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE));
            telemetry.addData("Axial Pos:", robot.clawAxial.getPosition());
            telemetry.update();
        }
        // FIXME
        /*
        // Step through each let of the path, ensuring that the Autonomous mode has not been stopped along the way
        robot.setClawPosition(robot.enable, 0, robot.enable, 0);

        sleep(800);

        // Drive forward for 3 seconds
        robot.driveFieldCentric(robot.DRIVE_SPEED, 0, 0);
        robot.encoderLift(robot.LIFT_SPEED,15,13);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            robot.encoderLiftFinish(false);
        }
        robot.encoderLiftFinish(true);

        sleep(500);

        robot.encoderLift(robot.LIFT_SPEED,-4,4);


        robot.setClawPosition(robot.pass, 0,robot.disable,0);
        sleep(800);
        robot.setClawPosition(robot.disable,0,robot.pass,0);

        robot.encoderLiftFinish(true);

        sleep(500);

        robot.encoderLift(robot.LIFT_SPEED,-11, 10);
        // Drive backward for 1 second
        robot.driveFieldCentric(-robot.DRIVE_SPEED, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            robot.encoderLiftFinish(false);
        }
        robot.encoderLiftFinish(true);

        sleep(500);

        robot.driveFieldCentric(0, robot.STRAFE_SPEED,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stop
        robot.driveFieldCentric(0, 0, 0);
        runtime.reset();
        telemetry.addData("Path", "Complete");
        telemetry.update();

         */
        sleep(1000);


    }
}
