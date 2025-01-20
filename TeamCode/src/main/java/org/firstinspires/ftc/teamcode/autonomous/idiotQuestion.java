package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name= "idiot", group ="Robot")
public class idiotQuestion extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);


    @Override
    public void runOpMode() {

        double x = 0;
        double y = 0;
        double bearing = 0;
        double slope = 0;
        boolean breaker = false;

        robot.init();
        ptFinder.init();

        /*
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

         */

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        aptDetector.visionInit();
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);

        waitForStart();

        /*
        robot.driveFieldCentric(0.5,0,0);

        robot.leftFrontDrive.setTargetPosition(1000);
        robot.leftBackDrive.setTargetPosition(1000);
        robot.rightFrontDrive.setTargetPosition(1000);
        robot.rightBackDrive.setTargetPosition(1000);

        while (robot.leftBackDrive.isBusy() ||
                robot.rightFrontDrive.isBusy() ||
                robot.rightBackDrive.isBusy() ||
                robot.elbowDrive.isBusy()) {
            telemetry.addData(String.valueOf(robot.leftFrontDrive.getTargetPosition()) + " " + String.valueOf(robot.leftBackDrive.getTargetPosition()) + " " + String.valueOf(robot.rightFrontDrive.getTargetPosition()) + " " + String.valueOf(robot.rightBackDrive.getTargetPosition()), "");
            telemetry.addData(String.valueOf(robot.leftFrontDrive.getCurrentPosition()) + " " + String.valueOf(robot.leftBackDrive.getCurrentPosition()) + " " + String.valueOf(robot.rightFrontDrive.getCurrentPosition()) + " " + String.valueOf(robot.rightBackDrive.getCurrentPosition()), "");
            telemetry.update();
        }

        sleep(2000);

         */

        robot.driveFieldCentric(0.2, 0, 0);
        while (opModeIsActive() && !breaker) {
            aptDetector.activeAPTscanner(-1);
            telemetry.update();
            if (aptDetector.targetFound) {
                x = Math.round(aptDetector.detectedTag.robotPose.getPosition().x*10)/10f;
                y = Math.round(aptDetector.detectedTag.robotPose.getPosition().y*10)/10f;
                breaker = true;
            }
            sleep(50);
        }
        robot.driveFieldCentric(0,0,0);
        sleep(200);

        telemetry.update();
        ptFinder.precise(x, y, -54.2, 39);
        telemetry.update();
        slope = ptFinder.exSlope;
        while (!ptFinder.atTargetPos(Math.round(x) , Math.round(y), -54.2, 39) && opModeIsActive()){
            aptDetector.activeAPTscanner(12);
            x = Math.round(aptDetector.detectedTag.robotPose.getPosition().x*10)/10f;
            y = Math.round(aptDetector.detectedTag.robotPose.getPosition().y*10)/10f;
            bearing = aptDetector.detectedTag.ftcPose.bearing;
            ptFinder.precise(x, y, -50, 39);
            if (Math.abs(ptFinder.exVectorY) < 0.02 && Math.abs(ptFinder.exVectorX) < 0.02){break;}
            ptFinder.bearingCorrection(bearing);
            telemetry.addData("Vectors: ", ptFinder.exVectorX + ", " + ptFinder.exVectorY);
            telemetry.addData("PreClip: ", ptFinder.clipVX + ", " + ptFinder.clipVY);
            telemetry.addData( "new slope: ", ptFinder.exSlope);
            telemetry.addData("original slope: ", slope);
            telemetry.update();
        }
        robot.driveRobotCentric(0,0,0);
        telemetry.clearAll();
        telemetry.addData("kms", "");
        telemetry.update();
        sleep(500);
        byte check = 0;
        while (opModeIsActive() && check != 6){
            aptDetector.activeAPTscanner(12);
            x = Math.round(aptDetector.detectedTag.robotPose.getPosition().x*10)/10f;
            y = Math.round(aptDetector.detectedTag.robotPose.getPosition().y*10)/10f;
            bearing = aptDetector.detectedTag.ftcPose.bearing;
            ptFinder.precise(x, y, -54.2, 39);
            ptFinder.bearingCorrection(bearing);
            telemetry.addData("Vectors: ", ptFinder.exVectorX + ", " + ptFinder.exVectorY);
            telemetry.addData("PreClip: ", ptFinder.clipVX + ", " + ptFinder.clipVY);
            telemetry.addData( "new slope: ", ptFinder.exSlope);
            telemetry.addData("original slope: ", slope);
            telemetry.update();
            if (ptFinder.atTargetPos(Math.round(x*10)/10. , Math.round(y*10)/10., -54.2, 39)){
                check++;
            }
        }
        robot.driveFieldCentric(0,0,0 );
        telemetry.clearAll();
        telemetry.addData("KMS", "");
        telemetry.update();
        sleep(5000);

        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.8 ) {
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
        }
        robot.setClawPosition(robot.pass,robot.superposition,robot.superposition);

        sleep(300);

        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60))); // score/ hook on
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.45  ) {
            telemetry.addData("SCORING", "...");
            telemetry.update();
            robot.setClawPosition(robot.pass,robot.superposition,robot.superposition);

        }

        telemetry.addData("DONE","!!");
    }
}
