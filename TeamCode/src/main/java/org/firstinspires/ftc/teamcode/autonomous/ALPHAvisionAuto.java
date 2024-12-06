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
        robot.driveFieldCentric(0.25,0,0);

        while (opModeIsActive() ) {
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
            robot.detectR(new Point(480,810), new Point(1440,270));
            //List<ColorBlobLocatorProcessor.Blob> blobS = robot.colorLocator.getBlobs();
            //telemetry.addData("BlOBS", String.valueOf(robot.blobS.isEmpty()));

            if (!robot.blobS.isEmpty()) {
                robot.driveFieldCentric(0,0,0);
                telemetry.addData("RUNG"," FOUND");
                robot.setClawPosition(robot.pass,0,robot.superposition);
                sleep(500);
                break;
            }


            sleep(20);
        }

        robot.driveFieldCentric(0.25,0,0);
        sleep(300);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(40)));
        sleep(200);
        while (robot.elbowDrive.isBusy()) {
            robot.setClawPosition(robot.pass,0,robot.superposition);
            telemetry.addData("GIVE ME", "LUNCH");
            if (robot.elbowDrive.getCurrentPosition() == (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(50))) {
                robot.setClawPosition(robot.disable,0,robot.pass);
            }
        }
        sleep(3000);
        telemetry.addData("DONE","!!");

    }
}
