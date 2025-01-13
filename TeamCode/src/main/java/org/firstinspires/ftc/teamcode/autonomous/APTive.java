package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@Autonomous(name= "APTive", group ="Robot")
public class APTive extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);


    @Override
    public void runOpMode() {

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
        while (!aptDetector.targetFound) {
            aptDetector.activeAPTscanner(-1);
        }
        robot.driveFieldCentric(0,0,0);
        sleep(200);

        aptDetector.activeAPTscanner(-1);
        ptFinder.linearEncoderMovement(aptDetector.cameraPosition.x, aptDetector.cameraPosition.y, -50.9, 40);
        while (!ptFinder.atTargetPos(aptDetector.cameraPosition.x , aptDetector.cameraPosition.y, -50.9, 40)){
            aptDetector.activeAPTscanner(-1);
            if (Math.round( ((40-aptDetector.cameraPosition.y) / (-50.9-aptDetector.cameraPosition.x)) *100)/100. != Math.round(ptFinder.exSlope*100)/100.){
                ptFinder.linearEncoderMovement(aptDetector.cameraPosition.x, aptDetector.cameraPosition.y, 2, 20);
                telemetry.addData("Changed course...", "new slope: " + ptFinder.exSlope);
            }
            ptFinder.bearingCorrection(aptDetector.detectedTag.ftcPose.bearing);
            if (!aptDetector.targetFound){
                telemetry.addData("Lost APT!!", "");
                break;
            }
        }
        robot.driveRobotCentric(0,0,0);

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
