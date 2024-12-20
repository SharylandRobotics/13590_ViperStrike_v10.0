package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

//opencv imports...
import org.opencv.core.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

@Autonomous(name = "Airshot Capture", group = "Robot")
@Disabled
public class AirshotEXPMNTL extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    // find the serial ID for the camera; indexes can change easily and I don't know which camera is which index (found out thru T&E)
    VideoCapture camera = new VideoCapture(0);

    @Override
    public void runOpMode() {
        // lmk if camera is not found
        if (!camera.isOpened()){
            telemetry.addData("Error, Could not open camera","");
        }
        // create Mat to store img
        Mat frame = new Mat();
        // Initialize all the hardware using the hardware class.
        robot.init();

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        waitForStart();
        // get a frame from the camera
        if (camera.read(frame)) {
            // save img
            Imgcodecs.imwrite("captured.jpg", frame);
            telemetry.addData("Saved Image!","");
        } else { telemetry.addData("Could not read image from camera...","");}
        // close camera
        camera.release();
        telemetry.addData("Image:", frame);
    }
}
