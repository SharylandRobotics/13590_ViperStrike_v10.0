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
import org.firstinspires.ftc.vision.opencv.ColorRange;

@Autonomous(name = "ALPHA auto by Vision", group = "Robot")
public class ALPHAvisionAuto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        // Initialize all the hardware using the hardware class. ONLY NEED TO BE DONE ONCE
        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        robot.visionInit(ColorRange.BLUE, true,0.5,1.0,1.0,1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.setClawPosition(robot.enable,0,robot.pass,0);

        robot.calibrateClaw(robot.ELBOW_FORWARD_PARALLEL);

        sleep(500);

        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(5)));
        //FIXME


        while (true) {
            robot.detectR();

            if (robot.blobS != null) { // check until a blob appears
                robot.driveFieldCentric(0,0,0); // stop movement
                break; // break out the loop
            }

            robot.driveFieldCentric(0.25,0,0); // move SLOWLY until blob appears
        }

        while (true) {
            robot.detectR();

            if (robot.blobS != null) { // check until a blob appears
                robot.driveFieldCentric(0,0,0); // stop movement
                break; // break out the loop
            }

            robot.driveFieldCentric(0,0.025,0); // move SLOWLY until blob appears
        }
        // ASSUME BLOB HAS BEEN FOUND !
        sleep(500);

        sleep(1000);
        stop();








    }
}
