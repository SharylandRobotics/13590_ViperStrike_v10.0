package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Drive by Time", group = "Robot")
@Disabled
public class AutoDriveByTime extends LinearOpMode {

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
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PARALLEL);

        while (opModeIsActive() ) {
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
    }
}
