package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AIRSHOT-MTM", group = "Experimental")
@Disabled
public class AirshotBasedEXP extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);
    VisionSoftware.colorDetector color = new VisionSoftware.colorDetector(this);
    VisionSoftware.aptDetector apTag = new VisionSoftware.aptDetector(this);
    PathfinderSoftware.pathFinder ptFinder =  new PathfinderSoftware.pathFinder(this);
    ElapsedTime runtime = new ElapsedTime();

    double countourToDistance(double contour){
        return Math.pow(contour,2) + contour + 0;
    } // missing a polynomial formula here ^^; x^2 + x + 0

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double extend;
        double elbowFactor = 0;
        boolean calibrate = false;

        Position robotPos = apTag.detectedTag.robotPose.getPosition();
        List<Double> samplePos = new ArrayList<>();

        double heading;
        // Initialize all the hardware using the hardware class.
        robot.init();
        ptFinder.init();

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);

        robot.extensionDrive.setTargetPosition(0);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(1.0);

        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
        color.visionInit("BLUE", false, -0.5, 0.5,0.5,-0.5);
        apTag.visionInit();

        // close portal until ready to airshot
        color.portalColor.stopStreaming();

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
        while (opModeIsActive() && runtime.seconds() < 0.45  ) {
            telemetry.addData("SCORING", "...");
            telemetry.update();
            robot.setClawPosition(robot.pass,robot.superposition,robot.superposition);

        }
        robot.driveFieldCentric(0,0,0);

        while (robot.elbowDrive.isBusy()) { // check for when to let go of specimen
            robot.setClawPosition(robot.pass,robot.pass,robot.superposition);

            if (robot.elbowDrive.getCurrentPosition() <= (int) (robot.COUNTS_PER_DEGREE * 100) ) {
                break;
            }
        }
        robot.setClawPosition(robot.disable,robot.pass ,robot.pass); // let go of specimen
        telemetry.addData("GET ELBOW ANGLE", robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE );
        telemetry.update();


        // TAKE AIRSHOT HERE
        robot.clawAxial.setPosition(0.67);
        sleep(500);
        color.portalColor.resumeStreaming();
        color.portalColor.setProcessorEnabled(color.primaryColorProcessor, true);
        color.portalColor.resumeLiveView();
        sleep(2500);
        color.portalColor.saveNextFrameRaw("airshot");
        Mat frame = Imgcodecs.imread("airshot.png");
        color.primaryColorProcessor.processFrame(frame, 10);
        sleep(500);

        color.primaryBlobList = color.primaryColorProcessor.getBlobs();
        for (ColorBlobLocatorProcessor.Blob b : color.primaryBlobList) {
            double distanceC = countourToDistance(b.getContourArea()); // done
            double heightB = robot.elbowDrive.getCurrentPosition()/robot.COUNTS_PER_DEGREE * 0.1; // height(in) per deg formula missing
            double depthA = Math.sqrt(Math.pow(distanceC,2) - Math.pow(heightB, 2)); // done
            samplePos.add(depthA + robotPos.y + 14); // add distance to list (idk if its x or y for the pose) and last # is robot length

        }
        for (Double num : samplePos){
            telemetry.addData(samplePos.indexOf(num) + "-th sample...", "\n Distance is :" + num +"\n");
            telemetry.update();
        }

    }
}
