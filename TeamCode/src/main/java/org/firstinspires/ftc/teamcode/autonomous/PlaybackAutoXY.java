package org.firstinspires.ftc.teamcode.autonomous;

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



    ElapsedTime runtime = new ElapsedTime();

    // values go like this : drive, strafe, turn, heading, armPos(in deg), apt.range
    double[] dataTable = {  2, 2, 0, 0, 0.4, 0.5, 3.0,  5,2,0,0,0.4,0.5,1.0,};
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
        robot.init();PathfinderSoftware ptfSoftware = new PathfinderSoftware(this);PathfinderSoftware.pathFinder ptFinder = ptfSoftware.new pathFinder(this);
        ptFinder.init();
        ptFinder.imu = this.hardwareMap.get(IMU.class, "imu");
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        telemetry.update();
        waitForStart();
        runtime.reset();

        for (int i = 0; i*indexSkip < dataTable.length; i++) { // for every set (of 6) of indexes, do...

            // current positions

            // target positions
            posX2 = dataTable[(i*indexSkip)];
            posY2 = dataTable[(i*indexSkip)+1];
            telemetry.addData(String.valueOf(i), "-th action");
            robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
            // go towards target

            // drive until you reach your target ...
            while (opModeIsActive() && !ptFinder.atTargetPos(posX1, posX2, posY1, posY2)) {
                aptDetector.activeAPTscanner(-1);
                if (aptDetector.targetFound) {
                    telemetry.addData("APT found","");
                    posX1 = aptDetector.detectedTag.robotPose.getPosition().x;
                    posY1 = aptDetector.detectedTag.robotPose.getPosition().y;
                } else {posX1 =0; posY1=0;}
                ptFinder.pathFind(posX1, posX2, posY1, posY2, dataTable[(i*indexSkip)+2]);
                telemetry.addData("driving...", "");
                telemetry.addData("X to nearest APT:", posX1);
                telemetry.addData("Y to nearest APT:", posY1);
                telemetry.update();
            }
        }

    }
}