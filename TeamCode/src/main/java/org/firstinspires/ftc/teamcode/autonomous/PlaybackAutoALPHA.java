package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@Autonomous(name = "PlayBack Auto ALPHA", group = "Experimental")
public class PlaybackAutoALPHA extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    ElapsedTime runtime = new ElapsedTime();

    // values go like this : drive, strafe, turn, heading, armPos(in deg), apt.range
    double[] dataTable = {};
    // how many indexes to skip to go to the next action
    int IS = 11; // just the # of things you print in Recorder ; short for Index Skip

    @Override
    public void runOpMode() {
        double heading;
        double rangeToAPT;
        double ancRangeToAPT;
        // Initialize all the hardware using the hardware class.
        robot.init();
        aptDetector.visionInit();
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(0);
        robot.leftBackDrive.setTargetPosition(0);
        robot.rightFrontDrive.setTargetPosition(0);
        robot.rightBackDrive.setTargetPosition(0);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        for (int i = 0; i*IS < dataTable.length; i++) { // for every set (of 6) of indexes, do...

            while (robot.leftFrontDrive.isBusy() ||
                robot.leftBackDrive.isBusy() ||
                robot.rightFrontDrive.isBusy() ||
                robot.rightBackDrive.isBusy() ||
                robot.elbowDrive.isBusy() )
            {
                telemetry.addData((robot.leftFrontDrive.isBusy() ? "LF Busy, " : " ") // telemetry which motor is busy
                        + (robot.leftBackDrive.isBusy() ? "LB Busy, " : "") +
                        (robot.rightFrontDrive.isBusy() ? "RF Busy, " : "") +
                        (robot.rightBackDrive.isBusy() ? "RB Busy, " : "") +
                        (robot.elbowDrive.isBusy() ? "ELBOW Busy, " : ""), "");
            }
                telemetry.addData(String.valueOf(i), "-th action");
                // set power to motors before encoders
                robot.driveFieldCentric(dataTable[i * IS], dataTable[(i * IS) + 1], dataTable[(i * IS) + 2]);

                // set encoder targets
                robot.leftFrontDrive.setTargetPosition((int) dataTable[(i * IS) + 7]);
                robot.leftBackDrive.setTargetPosition((int) dataTable[(i * IS) + 8]);
                robot.rightFrontDrive.setTargetPosition((int) dataTable[(i * IS) + 9]);
                robot.rightBackDrive.setTargetPosition((int) dataTable[(i * IS) + 10]);

                robot.elbowDrive.setTargetPosition((int) (dataTable[(i * IS) + 3] * robot.COUNTS_PER_DEGREE));
                robot.clawPinch.setPosition(dataTable[(i * IS) + 4]);
                robot.clawAxial.setPosition(dataTable[(i * IS) + 5]);

                // April Tag Logic Section
                aptDetector.activeAPTscanner(-1);
                rangeToAPT = aptDetector.detectedTag.ftcPose.range;
                ancRangeToAPT = dataTable[(i * IS) + 6];
                if (aptDetector.targetFound) {
                    if (Math.round(rangeToAPT / 0.1) * 0.1 == Math.round(ancRangeToAPT / 0.1) * 0.1) {
                        telemetry.addData("On Track", "...");
                    } else {
                        telemetry.addData("Off Course", "!!!\n Off by: " + Math.abs(ancRangeToAPT - rangeToAPT));
                    }
                } else {
                    telemetry.addData("No April Tag detected", "...\n Range should be: " + ancRangeToAPT);
                }
                telemetry.addData("Range to nearest APT:", rangeToAPT);
                telemetry.addData("Recorded Range to nearest APT:", ancRangeToAPT);
                telemetry.update();
                sleep(50);
        }

    }
}