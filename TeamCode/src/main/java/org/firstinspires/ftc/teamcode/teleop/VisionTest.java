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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

/*
 * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions
 *
 * Unlike a "color sensor" which determines the color of an object in the field of view, this "color locator"
 * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that match the requested color range.
 * These blobs can be further filtered and sorted to find the one most likely to be the item the user is looking for.
 *
 * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
 *   The ColorBlobLocatorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
 *   The matching pixels are then collected into contiguous "blobs" of pixels.  The outer boundaries of these blobs are called its "contour".
 *   For each blob, the process then creates the smallest possible rectangle "boxFit" that will fully encase the contour.
 *   The user can then call getBlobs() to retrieve the list of Blobs, where each Blob contains the contour and the boxFit data.
 *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are listed first.
 *
 * To aid the user, a colored boxFit rectangle is drawn on the camera preview to show the location of each Blob
 * The original Blob contour can also be added to the preview.  This is helpful when configuring the ColorBlobLocatorProcessor parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "Vision: test", group = "Robot")
public class VisionTest extends LinearOpMode {
    @SuppressLint("DefaultLocale")

    RobotHardware robot = new RobotHardware(this);

    ElapsedTime runtime = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode()
    {
        YawPitchRollAngles  yawAngles = robot.imu.getRobotYawPitchRollAngles();


        robot.visionInit();
        robot.init();
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeInInit())
        {
            yawAngles = robot.imu.getRobotYawPitchRollAngles(); // set orientation
            telemetry.addData("check preview, initialized", "... Camera Stream");
            robot.detectR(); // run camera
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

            robot.detectR(); // run camera



            sleep(50);
        }
    }
}
