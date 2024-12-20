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

package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@TeleOp(name = "Vision: test", group = "Robot")
public class VisionTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
        YawPitchRollAngles  yawAngles;

        robot.init();
        robot.visionInit("BLUE", false, -0.5, 0.5, 0.5, -0.5);
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(robot.colorLocator)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeInInit())
        {
            yawAngles = robot.imu.getRobotYawPitchRollAngles(); // set orientation
            telemetry.addData("check preview, initialized", "... Camera Stream");
            telemetry.addData("current orientation", String.valueOf(yawAngles));
            robot.detectR(new Point(480,810), new Point(1440,270)); // run camera

        }

        waitForStart();
        robot.imu.resetYaw(); // reset orientation
        runtime.reset();

        while (opModeIsActive()) { // once in play, print orientation

            /* the plan is to check orientation and draw a master north line in camera, thus giving it a sense
                of direction when thrown of course. This will also allow more precise camera-based movement.
                The line should move along with the camera: if the robot rotates 30deg ccw, the line will move
                30deg to the right projected(how it will display in 2d) onto the camera display.
            */
            yawAngles = robot.imu.getRobotYawPitchRollAngles(); // Check out the waters

            telemetry.addData("Yaw Angles:", String.valueOf(yawAngles)); // FIXME
            robot.detectR(new Point(480,810), new Point(1440,270)); // run camera
            telemetry.update();
            if(!robot.blobS.isEmpty() ) {
                telemetry.addData("RUNG SPOTTED", "!!!!");
            }
            sleep(50);
        }

    }
}
