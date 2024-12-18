package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@Autonomous(name = "PlayBack XY Auto ALPHA", group = "Robot")
public class PlaybackAutoXY extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);
    ElapsedTime runtime = new ElapsedTime();

    // values go like this : drive, strafe, turn, heading, armPos(in deg), apt.range
    double[] dataTable = {0,0,0,0,0.4,0.5,1.0,  5, 5, 0, 0, 0.4, 0.5, 3.0};
    // how many indexes to skip to go to the next action
    int indexSkip = 7; // just the # of things you print in Recorder

    @Override
    public void runOpMode() {
        double posX1 = 0;
        double posX2;
        double posY1 = 0;
        double posY2;
        // Initialize all the hardware using the hardware class.
        aptDetector.visionInit();
        robot.init();

        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        telemetry.addData(String.valueOf(robot.turnDirection(0, false)), "");
        telemetry.update();
        waitForStart();
        runtime.reset();

        for (int i = 0; i*indexSkip < dataTable.length; i++) { // for every set (of 6) of indexes, do...
            // current positions
            if (aptDetector.targetFound) {
                posX1 = aptDetector.detectedTag.robotPose.getPosition().x;
                posY1 = aptDetector.detectedTag.robotPose.getPosition().y;
            } else {posX1 =0; posY1=0;}
            // target positions
            posX2 = dataTable[(i*indexSkip)];
            posY2 = dataTable[(i*indexSkip)+1];
            telemetry.addData(String.valueOf(i), "-th action");
            robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
            // go towards target
            ptFinder.pathFind(posX1, posX2, posY1, posY2, dataTable[(i*indexSkip)+2], robot.heading);

            // drive until you reach your target ...
            while (opModeIsActive() && !ptFinder.atTargetPos(posX1, posX2, posY1, posY2)) {
                telemetry.addData("driving...", "");
                telemetry.update();
            }
        }

    }
}