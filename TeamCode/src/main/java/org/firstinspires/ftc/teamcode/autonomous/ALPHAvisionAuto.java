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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.List;

@Autonomous(name = "ALPHA auto by Vision", group = "Robot")
public class ALPHAvisionAuto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        YawPitchRollAngles yawPitchRoll;
        double heading;
        // Initialize all the hardware using the hardware class.
        robot.init();
        robot.visionInit("BLUE", true, -0.6, 0.6, 0.6, -0.6);
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15)));
        robot.driveFieldCentric(0.6,0,0);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.8 ) {
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
        }

        robot.driveFieldCentric(0,0,0);
        robot.setClawPosition(robot.pass,0,robot.superposition);

        sleep(300);
        robot.driveFieldCentric(0.3,0,0);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60)));
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2  ) {
            telemetry.addData("STABILIZING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);
        while (robot.elbowDrive.isBusy()) {
            robot.setClawPosition(robot.pass,0,robot.superposition);

            if (robot.elbowDrive.getCurrentPosition() == (int) (robot.COUNTS_PER_DEGREE * 90) ) {
                break;
            }
        }
        robot.setClawPosition(robot.disable,0,robot.pass);
        telemetry.addData("GIVE ME", robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE );
        telemetry.update();
        sleep(200);
        robot.setClawPosition(robot.enable,0, robot.pass);
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_COLLAPSED);

        robot.driveFieldCentric(-0.6,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.01) {
            telemetry.addData("BACKING UP", "...");
            telemetry.update();
        }

        sleep(500);

        robot.driveFieldCentric(-0.8,0.8,0);
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PARALLEL);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.6) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);
        sleep(3000);
        robot.turnUntil(179.9);

        telemetry.addData("DONE","!!");

    }
}
