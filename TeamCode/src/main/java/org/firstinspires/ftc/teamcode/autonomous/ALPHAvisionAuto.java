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

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "ALPHA auto by Vision", group = "Robot")
public class ALPHAvisionAuto extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize all the hardware using the hardware class. ONLY NEED TO BE DONE ONCE
        robot.visionInit();
        robot.init();
        robot.stopNreset();
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Grab Preload Specimen
        robot.setClawPosition(robot.enable, 0, robot.enable, 0);

        sleep(800);

        // Drive forward for 3 seconds
        robot.driveFieldCentric(robot.DRIVE_SPEED, 0, 0);
        // Enable Lift
        robot.encoderLift(robot.LIFT_SPEED,15,13);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            // Check if Lift is Done
            robot.encoderLiftFinish(false);
        }
        // Wait until Lift is Done
        robot.encoderLiftFinish(true);

        sleep(500);

        robot.driveFieldCentric(0.1, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        sleep(500);

        robot.encoderLift(robot.LIFT_SPEED, -15,13);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Minor Leg 2.1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.setClawPosition(robot.pass,0,robot.disable,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Minor Leg 2.2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.setClawPosition(robot.disable,0,robot.pass,0);
        robot.driveFieldCentric(-robot.DRIVE_SPEED,0,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        sleep(500);

        robot.driveFieldCentric(0,robot.STRAFE_SPEED,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.encoderLiftFinish(true);
        robot.driveFieldCentric(0,0,robot.TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        sleep(500);

        robot.driveFieldCentric(0.1,0,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 6: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        sleep(500);

        while (true) {
            robot.detectR();

            if (!robot.blobs.isEmpty()) { // check until a blob appears
                robot.driveFieldCentric(0,0,0); // stop movement
                break; // break out the loop
            }

            robot.driveFieldCentric(0,0.025,0); // move SLOWLY until blob appears
        }
        // ASSUME BLOB HAS BEEN FOUND
        sleep(500);

        robot.setClawPosition(robot.disable,0,robot.superposition,0);

        sleep(500);

        robot.driveFieldCentric(0.1,0,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 7: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.setClawPosition(robot.enable,0,robot.pass,0);

        sleep(500);

        robot.encoderLift(robot.LIFT_SPEED,15,13);
        robot.driveFieldCentric(0,0,-robot.TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 8: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        sleep(500);

        robot.driveFieldCentric(0,robot.STRAFE_SPEED,0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.encoderLiftFinish(true);

        sleep(1000);
        stop();






    }
}
