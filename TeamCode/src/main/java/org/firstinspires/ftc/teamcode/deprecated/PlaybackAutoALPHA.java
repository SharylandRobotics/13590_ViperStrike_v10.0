package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;
@Disabled
@Autonomous(name = "PlayBack Auto ALPHA", group = "Experimental")
public class PlaybackAutoALPHA extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    ElapsedTime runtime = new ElapsedTime();

    // values go like this : drive, strafe, turn, heading, armPos(in deg), apt.range
    double[] dataTable = {-0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, 0.8998733162879944, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 15, 13, 16, 12, 0.8748416304588318, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 150, 154, 123, 118, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 279, 296, 229, 229, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 315, 331, 245, 250, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 332, 348, 256, 263, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 336, 351, 259, 266, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 336, 350, 259, 266, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 336, 350, 258, 266, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 336, 350, 258, 266, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 336, 350, 258, 266, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 336, 350, 258, 266, -0.0, 1.0632868826389315, 0.0, 0.0, 0.4, 0.0, -1, 358, 331, 237, 286, -0.0, 1.1, 0.0, 0.0, 0.4, 0.0, -1, 493, 177, 106, 434, -0.0, 1.1, 0.0, 0.0, 0.4, 0.0, -1, 641, 12, -38, 595, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 791, -157, -183, 762, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 810, -183, -201, 792, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 824, -200, -212, 817, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -206, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 825, -207, -221, 822, -0.0, -0.30152538120746614, 0.0, 0.0, 0.4, 0.0, -1, 822, -203, -215, 818, -0.0, -0.7879742026329041, 0.0, 0.0, 0.4, 0.0, -1, 771, -157, -160, 783, -0.0, -1.1, 0.0, 0.0, 0.4, 0.0, -1, 629, -15, -7, 661, -0.0, -1.1, 0.0, 0.0, 0.4, 0.0, -1, 470, 152, 165, 509, -0.0, -1.1, 0.0, 0.0, 0.4, 0.0, -1, 310, 323, 332, 362, -0.0, -0.00782046588137746, 0.0, 0.0, 0.4, 0.0, -1, 140, 507, 510, 199, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 17, 635, 635, 78, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -14, 654, 661, 59, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -40, 667, 679, 58, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -51, 673, 690, 55, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -53, 672, 689, 54, -0.0, 0.0, -0.12392397224903107, 0.0, 0.4, 0.0, -1, -55, 674, 687, 55, -0.0, 0.0, -0.5327746272087097, 0.0, 0.4, 0.0, -1, -79, 695, 664, 76, -0.0, 0.0, -0.5744940638542175, 0.0, 0.4, 0.0, -1, -171, 795, 569, 170, -0.0, 0.0, -0.5828379392623901, 0.0, 0.4, 0.0, -1, -301, 922, 448, 295, -0.0, 0.0, -0.5911818146705627, 0.0, 0.4, 0.0, -1, -427, 1047, 329, 416, -0.0, 0.0, -0.5911818146705627, 0.0, 0.4, 0.0, -1, -549, 1167, 213, 536, -0.0, 0.0, -0.5911818146705627, 0.0, 0.4, 0.0, -1, -680, 1299, 86, 667, -0.0, 0.0, -0.15729954838752747, 0.0, 0.4, 0.0, -1, -814, 1429, -39, 799, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -910, 1524, -125, 896, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -926, 1541, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1541, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1541, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, -0.026177021488547326, 0.0, 0.0, 0.4, 0.0, -1, -927, 1540, -140, 911, -0.0, -0.3474167943000794, 0.0, 0.0, 0.4, 0.0, -1, -920, 1547, -134, 917, -0.0, -0.7053697049617768, 0.0, 0.0, 0.4, 0.0, -1, -861, 1617, -86, 954, -0.0, -0.7604393482208253, 0.0, 0.0, 0.4, 0.0, -1, -734, 1768, 36, 1058, -0.0, -0.1822077825665474, 0.0, 0.0, 0.4, 0.0, -1, -606, 1919, 175, 1183, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -504, 2031, 278, 1284, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -471, 2062, 292, 1302, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -447, 2082, 301, 1316, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2092, 313, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, -434, 2091, 314, 1322, -0.0, 0.0, 0.0, 0.0, 0.4, 0.0, -1, 0, 0, 0, 0, };
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

        for (int i = 0; i*IS < dataTable.length; i++) { // for every set (of 6) of indexes, do...


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
                if (//!(dataTable[i * IS]== 0) && !(dataTable[(i * IS) + 1]==0) && !(dataTable[(i * IS) + 2]==0))
                        true)
                {
                    while ((robot.leftFrontDrive.getCurrentPosition() != (int) dataTable[(i * IS) + 7]) ||
                        (robot.leftBackDrive.getCurrentPosition() != (int) dataTable[(i * IS) + 8]) ||
                        (robot.rightFrontDrive.getCurrentPosition() != (int) dataTable[(i * IS) + 9]) ||
                        (robot.rightBackDrive.getCurrentPosition() != (int) dataTable[(i * IS) + 10]))
                    {
                        telemetry.addData((robot.leftFrontDrive.isBusy() ? "LF Busy, " : " ") // telemetry which motor is busy
                            + (robot.leftBackDrive.isBusy() ? "LB Busy, " : "") +
                            (robot.rightFrontDrive.isBusy() ? "RF Busy, " : "") +
                            (robot.rightBackDrive.isBusy() ? "RB Busy, " : "") +
                            (robot.elbowDrive.isBusy() ? "ELBOW Busy, " : ""), "");
                        telemetry.addData(String.valueOf((int) dataTable[(i * IS) + 7]) + " " + String.valueOf((int) dataTable[(i * IS) + 8]) + " " + String.valueOf((int) dataTable[(i * IS) + 9]) + " " + String.valueOf((int) dataTable[(i * IS) + 10]), "");
                        telemetry.addData(String.valueOf(robot.leftFrontDrive.getCurrentPosition()) + " " + String.valueOf(robot.leftBackDrive.getCurrentPosition()) + " " + String.valueOf(robot.rightFrontDrive.getCurrentPosition()) + " " + String.valueOf(robot.rightBackDrive.getCurrentPosition()), "");
                        telemetry.addData(String.valueOf(i), "-th action");
                        telemetry.update();
                    }
                }
        }

    }
}