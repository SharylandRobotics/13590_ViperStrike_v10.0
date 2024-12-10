/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;
import org.opencv.core.Point;

@Autonomous(name = "ALPHA auto by Vision", group = "Robot")
public class ALPHAvisionAuto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    VisionSoftware.colorDetector colorDetector = new VisionSoftware.colorDetector(this);

    @Override
    public void runOpMode() {

        double secondsToScan = 0;
        // Initialize all the hardware using the hardware class.
        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        robot.driveFieldCentric(0.6,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.8 ) {
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
        }
        robot.driveFieldCentric(0,0,0);
        robot.setClawPosition(robot.pass,robot.superposition,robot.superposition);

        sleep(300);

        robot.driveFieldCentric(0.3,0,0);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60))); // score/ hook on
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2  ) {
            telemetry.addData("SCORING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        while (robot.elbowDrive.isBusy()) { // check for when to let go of specimen
            robot.setClawPosition(robot.pass,robot.pass,robot.superposition);

            if (robot.elbowDrive.getCurrentPosition() == (int) (robot.COUNTS_PER_DEGREE * 90) ) {
                break;
            }
        }
        robot.setClawPosition(robot.disable,robot.pass ,robot.pass); // let go of specimen
        telemetry.addData("GET ELBOW ANGLE", robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE );
        telemetry.update();

        sleep(200);

        robot.setClawPosition(robot.enable,robot.pass, robot.pass); // close to not hit bar
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PARALLEL); // lower to position for pickup

        robot.driveFieldCentric(-0.5,0,0); // back away to not hit arm
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_BACKWARD_PARALLEL);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.15) {
            telemetry.addData("BACKING UP", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(500);
        /*
        robot.setDrivePower(0.43,-1.0,-1.0,0.43); // strafe to OZ
        robot.setClawPosition(robot.disable,0,robot.pass);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.6) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(3000);

        robot.driveFieldCentric(0,0,-1.0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            telemetry.addData("TURNING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(10000); // TEST THIS BEFORE MOVING ON  ^^^^^^^^^

        colorDetector.visionInit("BLUE",true, colorDetector.colorLocator, -1,0.6255,0.5,-0.625); // scan for the specimen
        robot.driveFieldCentric(0,-0.625,0); // USE THIS SAME SPEED FOR secondsToScan
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
            secondsToScan = runtime.seconds();
            colorDetector.detectR(new Point(480,810), new Point(1440,270), "PRIMARY");
            if (!colorDetector.blobS.isEmpty()) {
                robot.driveFieldCentric(0,0,0);
                secondsToScan = runtime.seconds();
                break;
            }
        }

        sleep(300);

        robot.driveFieldCentric(0,0.625,0); // REVERSE THE DRIVING FROM ABOVE ^^^
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < secondsToScan) {
            telemetry.addData("MOVING BACK", "...");
            telemetry.update();
        }

        robot.driveFieldCentric(0.625,0,0); // move up to grab specimen
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            telemetry.addData("DOCKING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.setClawPosition(robot.enable,0,robot.enable); // grab specimen
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR + robot.angleConvert(15))); // lift arm
        robot.setDrivePower(1.0,-1.0,-0.43,0.43); // strafe back to rung
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.6) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(0.6,0,0); // move up/reverse of back away
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.01) {
            telemetry.addData("MOVING UP", "...");
            telemetry.update();
        }

        robot.driveFieldCentric(0.3,0,0); // hook onto rung/score
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR + robot.angleConvert(60)));
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2  ) {
            telemetry.addData("SCORING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        while (robot.elbowDrive.isBusy()) { // check for when to let go of specimen
            robot.setClawPosition(robot.pass,0,robot.superposition);

            if (robot.elbowDrive.getCurrentPosition() == (int) (robot.COUNTS_PER_DEGREE * 180) ) {// test and edit this #
                break;
            }
        }
        robot.setClawPosition(robot.disable,0,robot.pass); // let go of specimen
        telemetry.addData("GET ELBOW ANGLE", robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE );
        telemetry.update();

        */
        // bring arm down and perpendicularize claw before this, also assume that this is after your FIRST/PRELOADED specimen
        robot.driveFieldCentric(0,1.0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.6) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.6) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.setDrivePower(0,-1,-1,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4){
            telemetry.addData("DIAGONAL", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(-1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.625) {
            telemetry.addData("DRIVING BACK", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.625) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(0,1.0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(-1,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.625) {
            telemetry.addData("DRIVING BACK", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(1,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.625) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(0,0.3,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.1) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(-1,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.625) {
            telemetry.addData("DRIVING BACK", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        telemetry.addData("DONE","!!");

    }
}
