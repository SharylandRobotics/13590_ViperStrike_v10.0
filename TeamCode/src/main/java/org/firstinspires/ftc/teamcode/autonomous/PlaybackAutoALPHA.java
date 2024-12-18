package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@Autonomous(name = "PlayBack Auto ALPHA", group = "Robot")
public class PlaybackAutoALPHA extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    ElapsedTime runtime = new ElapsedTime();

    // values go like this : drive, strafe, turn, heading, armPos(in deg), apt.range
    double[] dataTable = {};
    // how many indexes to skip to go to the next action
    int indexSkip = 7; // just the # of things you print in Recorder

    @Override
    public void runOpMode() {
        double heading;
        double secondsToScan = 0;
        // Initialize all the hardware using the hardware class.
        robot.init();
        aptDetector.visionInit();
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        for (int i = 0; i*7 < dataTable.length; i++) { // for every set (of 6) of indexes, do...
            telemetry.addData(String.valueOf(i), "-th action");
            robot.driveFieldCentric(dataTable[i * 7], dataTable[(i * 7) + 1], dataTable[(i * 7) + 2]);
            robot.elbowDrive.setTargetPosition((int) (dataTable[(i * 7) + 3] * robot.COUNTS_PER_DEGREE));
            robot.clawPinch.setPosition(dataTable[(i * 7) + 4]);
            robot.clawAxial.setPosition(dataTable[(i * 7) + 5]);
            aptDetector.activeAPTscanner(-1);
            if (aptDetector.targetFound) {
                if (Math.round(aptDetector.detectedTag.ftcPose.range) == Math.round(dataTable[(i * 7) + 6])) {
                    telemetry.addData("On Track", "...");
                } else {
                    telemetry.addData("Off Course", "!!!");
                }
            } else {
                telemetry.addData("No April Tag detected", "...");


            }
            telemetry.update();
            sleep(50);
        }

    }
}