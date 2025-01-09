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
    double[] dataTable = {-0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, 0.5243982672691345, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -13, -12, -16, -15, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -137, -133, -138, -131, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -357, -353, -358, -348, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -586, -581, -584, -596, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -880, -878, -880, -860, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1111, -1108, -1107, -1083, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1332, -1329, -1328, -1300, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1583, -1591, -1590, -1558, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1815, -1818, -1806, -1775, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -1834, -1820, -1792, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -1834, -1820, -1792, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -1834, -1820, -1792, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -1834, -1820, -1792, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -1834, -1820, -1792, -0.0, -1.0266094982624054, 0.0, 0.0, 0.4, 0.0, -1, -1802, -1854, -1848, -1772, -0.0, -1.1, 0.0, 0.0, 0.4, 0.0, -1, -1612, -2051, -2046, -1574, -0.0, -1.1, 0.0, 0.0, 0.4, 0.0, -1, -1343, -2314, -2308, -1318, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1101, -2563, -2548, -1077, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1087, -2579, -2562, -1060, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1087, -2579, -2562, -1060, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1087, -2579, -2562, -1060, -0.0, 0.035319588333368304, 0.0, 0.0, 0.4, 0.0, -1, -1088, -2578, -2561, -1061, -0.0, 1.1, 0.0, 0.0, 0.4, 0.0, -1, -1118, -2555, -2530, -1082, -0.0, 1.1, 0.0, 0.0, 0.4, 0.0, -1, -1317, -2355, -2333, -1273, -0.0, 0.503411826491356, 0.0, 0.0, 0.4, 0.0, -1, -1545, -2125, -2111, -1494, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1733, -1927, -1928, -1684, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1746, -1912, -1917, -1700, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1746, -1912, -1917, -1700, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1746, -1912, -1917, -1700, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1746, -1912, -1917, -1700, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1746, -1912, -1917, -1700, 0.6412127017974854, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1762, -1923, -1932, -1710, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1907, -2060, -2073, -1848, 1.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -2129, -2283, -2298, -2065, 0.12389151006937027, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -2405, -2562, -2571, -2335, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -2591, -2764, -2757, -2534, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -2600, -2773, -2764, -2543, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -2600, -2773, -2764, -2543, -0.9499691128730774, -0.03535530902445316, 0.0, 0.0, 0.4, 0.0, -1, -2572, -2756, -2736, -2520, -1.0, -0.1363163694739342, 0.0, 0.0, 0.4, 0.0, -1, -2335, -2530, -2516, -2285, -0.4576795995235443, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -2027, -2280, -2277, -1982, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1844, -2125, -2133, -1791, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1775, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1775, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1775, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1775, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1775, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1776, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, -2113, -2125, -1776, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -1831, 0, 0, 0, };
    // how many indexes to skip to go to the next action
    int IS = 11; // just the # of things you print in Recorder ; short for Index Skip

    @Override
    public void runOpMode() {
        double heading;
        double rangeToAPT = -1;
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

        robot.leftFrontDrive.setPower(0.5);
        robot.leftFrontDrive.setTargetPosition(-1000);

        for (int i = 0; i*IS < dataTable.length; i++) { // for every set (of 6) of indexes, do...

            while ((robot.leftFrontDrive.getCurrentPosition() != robot.leftFrontDrive.getTargetPosition()) ||
                    (robot.leftBackDrive.getCurrentPosition() != robot.leftBackDrive.getTargetPosition()) ||
                    (robot.rightFrontDrive.getCurrentPosition() != robot.rightFrontDrive.getTargetPosition()) ||
                    (robot.rightBackDrive.getCurrentPosition() != robot.rightBackDrive.getTargetPosition()))
            {
                telemetry.addData((robot.leftFrontDrive.isBusy() ? "LF Busy, " : " ") // telemetry which motor is busy
                        + (robot.leftBackDrive.isBusy() ? "LB Busy, " : "") +
                        (robot.rightFrontDrive.isBusy() ? "RF Busy, " : "") +
                        (robot.rightBackDrive.isBusy() ? "RB Busy, " : "") +
                        (robot.elbowDrive.isBusy() ? "ELBOW Busy, " : ""), "");
                telemetry.addData(String.valueOf((int) dataTable[(i * IS) + 7]) + " "+String.valueOf((int) dataTable[(i * IS) + 8]) +  " "+String.valueOf((int) dataTable[(i * IS) + 9]) + " "+String.valueOf((int) dataTable[(i * IS) + 10]), "");
                telemetry.addData(String.valueOf(robot.leftFrontDrive.getCurrentPosition()) +" "+String.valueOf(robot.leftBackDrive.getCurrentPosition())+ " "+String.valueOf(robot.rightFrontDrive.getCurrentPosition())+" "+String.valueOf(robot.rightBackDrive.getCurrentPosition()),"");
                telemetry.update();
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
                ancRangeToAPT = dataTable[(i * IS) + 6];
                if (aptDetector.targetFound) {
                    rangeToAPT = aptDetector.detectedTag.ftcPose.range;
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