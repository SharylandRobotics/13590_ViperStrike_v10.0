package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@Autonomous(name = "APT auto", group = "Robot")
@Disabled
public class APTautoALPHA extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        YawPitchRollAngles yawPitchRoll;
        double heading;
        // Initialize all the hardware using the hardware class.
        robot.init();
        aptDetector.visionInit();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);
        while (opModeInInit()) {
            aptDetector.activeAPTscanner(-1);
        }

        waitForStart();
        runtime.reset();
        robot.driveFieldCentric(0.15,0,0);

        while (opModeIsActive() ) {
            yawPitchRoll = robot.imu.getRobotYawPitchRollAngles(); // set orientation
            heading = yawPitchRoll.getYaw(); // set Yaw angle

            aptDetector.activeAPTscanner(-1);
            if (aptDetector.targetFound) {robot.driveFieldCentric(0,0,0);}

            telemetry.addData("current orientation", String.valueOf(yawPitchRoll));
            telemetry.addData("current yaw", String.valueOf(heading));
            telemetry.update();
        }
        // FIXME
    }
}
