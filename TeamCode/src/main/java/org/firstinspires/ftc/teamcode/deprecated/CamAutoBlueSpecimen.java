package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

@Autonomous(name= "BlueSpecimenCam", group ="Blue")
public class CamAutoBlueSpecimen extends LinearOpMode {

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

        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        aptDetector.visionInit();
        aptDetector.portalAPT.setProcessorEnabled(aptDetector.APTprocessor, true);

        waitForStart();

        runtime.reset();

        sleep(300);


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.45  ) {
            telemetry.addData("SCORING", "...");
            telemetry.update();
            robot.setClawPosition(robot.pass,robot.superposition,robot.superposition);

        }

        robot.driveFieldCentric(0.5, 0, 0);
        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        while (opModeIsActive() && !breaker) {
            aptDetector.activeAPTscanner(-1);
            telemetry.update();
            if (aptDetector.targetFound) {
                x = aptDetector.detectedTag.robotPose.getPosition().x;
                y = aptDetector.detectedTag.robotPose.getPosition().y;
                breaker = true;
            }
            telemetry.addData("check",x + ", " + y);
            sleep(50);
        }
        robot.driveFieldCentric(0,0,0);
        sleep(100);

        byte check = 0;
        ptFinder.linearEncoderMovement(x, y, -9.5, 38, (float) 0.05, 0.01f);
        slope = ptFinder.exSlope;
        while (!ptFinder.atTargetPos(Math.round(x) , Math.round(y), -9.5, 38) && opModeIsActive() || check < 3){
            aptDetector.activeAPTscanner(12);
            x = aptDetector.detectedTag.robotPose.getPosition().x;
            y = aptDetector.detectedTag.robotPose.getPosition().y;
            bearing = aptDetector.detectedTag.ftcPose.bearing;
            ptFinder.linearEncoderMovement(x, y, -9.5, 38, 0.07f, 0.025f);
            if (ptFinder.atTargetPos(Math.round(x*10)/10. , Math.round(y*10)/10., x, 38) || (ptFinder.exVectorX < 0.07 || ptFinder.exVectorY < 0.07)){
                check++;
            }            ptFinder.bearingCorrection(bearing);
            telemetry.addData("Vectors: ", ptFinder.exVectorX + ", " + ptFinder.exVectorY);
            telemetry.addData("PreClip: ", ptFinder.clipVX + ", " + ptFinder.clipVY);
            telemetry.addData( "new slope: ", ptFinder.exSlope);
            telemetry.addData("original slope: ", slope);
            telemetry.update();
        }
        robot.driveRobotCentric(0,0,0);
        telemetry.clearAll();
        telemetry.addData("leg 1", "");
        telemetry.update();

        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60))); // score/ hook on

        sleep(200);

        robot.driveFieldCentric(-0.8,0,0);
        sleep(200);
        robot.driveFieldCentric(0,0,0);

        check = 0;
        ptFinder.linearEncoderMovement(x, y, -32, 39, (float) 0.05, 0.01f);
        slope = ptFinder.exSlope;
        while (!ptFinder.atTargetPos(Math.round(x) , Math.round(y), -32, 34.5) && opModeIsActive() || check < 3){
            aptDetector.activeAPTscanner(12);
            x = aptDetector.detectedTag.robotPose.getPosition().x;
            y = aptDetector.detectedTag.robotPose.getPosition().y;
            bearing = aptDetector.detectedTag.ftcPose.bearing;
            ptFinder.linearEncoderMovement(x, y, -32, 34.5, 0.07f, 0.025f);
            if (ptFinder.atTargetPos(Math.round(x) , Math.round(y), -32, y)){
                check++;
            }            ptFinder.bearingCorrection(bearing);
            telemetry.addData("Vectors: ", ptFinder.exVectorX + ", " + ptFinder.exVectorY);
            telemetry.addData("PreClip: ", ptFinder.clipVX + ", " + ptFinder.clipVY);
            telemetry.addData( "new slope: ", ptFinder.exSlope);
            telemetry.addData("original slope: ", slope);
            telemetry.update();
        }
        robot.driveRobotCentric(0,0,0);
        telemetry.clearAll();
        telemetry.addData("leg 2", "");
        telemetry.update();

        check = 0;
        ptFinder.linearEncoderMovement(x, y, -33, 17, (float) 0.05, 0.01f);
        slope = ptFinder.exSlope;
        while (!ptFinder.atTargetPos(Math.round(x) , Math.round(y), -33, 17) && opModeIsActive() || check < 3){
            aptDetector.activeAPTscanner(12);
            x = aptDetector.detectedTag.robotPose.getPosition().x;
            y = aptDetector.detectedTag.robotPose.getPosition().y;
            bearing = aptDetector.detectedTag.ftcPose.bearing;
            ptFinder.linearEncoderMovement(x, y, -33, 17, 0.07f, 0.025f);
            if (ptFinder.atTargetPos(Math.round(x) , Math.round(y), -33, 17)){
                check++;
            }            ptFinder.bearingCorrection(bearing);
            telemetry.addData("Vectors: ", ptFinder.exVectorX + ", " + ptFinder.exVectorY);
            telemetry.addData("PreClip: ", ptFinder.clipVX + ", " + ptFinder.clipVY);
            telemetry.addData( "new slope: ", ptFinder.exSlope);
            telemetry.addData("original slope: ", slope);
            telemetry.update();
        }
        robot.driveRobotCentric(0,0,0);
        telemetry.clearAll();
        telemetry.addData("leg 2", "");
        telemetry.update();


        telemetry.addData("DONE","!!");
    }
}
