package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.VisionSoftware;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name= "LL", group ="LL")
public class LLcoolJ extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    VisionSoftware.aptDetector aptDetector = new VisionSoftware.aptDetector(this);
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        //limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");

        telemetry.setMsTransmissionInterval(11);

        //limelight.pipelineSwitch(1);

        final double MtoIN = 39.3701;
        double x = 0;
        double y = 0;
        double bearing = 0;
        Pose3D botPose;
        double ozX = 20;
        double ozY = 50;
        robot.init();
        ptFinder.init();


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



        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);

        robot.extensionDrive.setTargetPosition(0);

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(1.0);

        //limelight.start();
        robot.setClawPosition(robot.enable, robot.superposition, robot.enable);
        waitForStart();



        robot.driveFieldCentric(0.6,0,0);
        robot.encoderFieldCentric(27,0,0);
        sleep(100);
        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        robot.extensionDrive.setTargetPosition((int) (1.5*robot.COUNTS_PER_MOTOR_REV));
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }
        sleep(1000);
        robot.clawAxial.setPosition(robot.CLAW_MID);

        while (robot.elbowDrive.isBusy()){
            telemetry.addData("raising","");
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }
        while (robot.extensionDrive.isBusy()){
            telemetry.addData("raising","");
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }

        sleep(1000);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60))); // score/ hook on
        while (robot.elbowDrive.isBusy()){
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ARM_COUNTS_PER_DEGREE*100))){
                robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
                robot.clawAxial.setPosition(robot.CLAW_MID);
                robot.extensionDrive.setTargetPosition(0);
                break;
            }
        }

        sleep(5000);

        // not 90 to save time later on while angling to pick up sample
        robot.encoderFieldCentric(0,0, 70);
        // angle of claw
        robot.clawYaw.setPosition(robot.YAW_LEFT-0.2);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        // set arm to parallel
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_ANGLE_OFFSET*robot.ARM_COUNTS_PER_DEGREE));

        // drive to pos
        ptFinder.tryAgain(x,y,(x+21),(y));
        robot.encoderFieldCentric(0, 16,0);
        // extend in advance to not move
        robot.extensionDrive.setTargetPosition((int) (4*robot.COUNTS_PER_MOTOR_REV));
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        // begin pick-up/drop sequence -->
        robot.setClawPosition(robot.disable, robot.pass, robot.pass);
        robot.elbowDrive.setTargetPosition((int) (25*robot.ARM_COUNTS_PER_DEGREE));
        while (robot.elbowDrive.isBusy()){
            robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            telemetry.addData("Busy","");
            telemetry.update();
        }
        // align claw with arm
        robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
        // close claw / grip sample
        robot.setClawPosition(robot.enable, robot.pass, robot.pass);
        sleep(100);
        // bring arm slightly up
        robot.elbowDrive.setTargetPosition((int) (30*robot.ARM_COUNTS_PER_DEGREE));
        // rotate claw
        robot.setClawPosition(robot.pass, robot.disable, robot.pass);
        // rotate bot to place in OZ
        robot.driveFieldCentric(0,0, 1.0);
        robot.encoderFieldCentric(0,0,60);
        // drop sample
        robot.setClawPosition(robot.disable, robot.pass, robot.pass);
        sleep(100);
        // rotate claw
        robot.setClawPosition(robot.pass, robot.enable, robot.pass);
        // rotate bot back to position
        robot.driveFieldCentric(0,0, -1.0);
        robot.encoderFieldCentric(0,0,-60);
        // move bot to next sample
        robot.driveFieldCentric(0, 1.0, 0);
        robot.encoderFieldCentric(0, 6, 0);
        // repeat ^

        // begin 1st specimen scoring off sidewall
        sleep(5000);

        // get arm in position
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PARALLEL));
        robot.setClawPosition(robot.pass, robot.pass, robot.superposition);

        // straighten up
        robot.driveFieldCentric(-0.5, 0, -0.5);
        robot.encoderFieldCentric(-5, 0, -30);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        // drive to meet
        robot.driveFieldCentric(0, 0.3, 0);
        robot.encoderFieldCentric(0, 1,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        robot.setClawPosition(robot.enable, robot.pass, robot.pass);
        sleep(100);

        ptFinder.tryAgain(-50, 20, 9, 37);
        robot.driveFieldCentric(robot.drivePower, robot.strafePower, 0.6);
        robot.encoderFieldCentric(59, 17, -90);

        // score here

        // return to OZ
        ptFinder.tryAgain(x, y, ozX, ozY);
        robot.encoderFieldCentric((ozX-x), (ozY-y), 0);
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_BACKWARD_PARALLEL);
        robot.setClawPosition(robot.disable, robot.superposition, robot.superposition);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        while (robot.elbowDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.driveFieldCentric(-0.2,0,0);
        robot.encoderFieldCentric(-4,0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        robot.setClawPosition(robot.enable, robot.pass, robot.superposition);
        sleep(100);

        // go back to score


        // begin scoring sequence -> (after picking up all 3 samples)

        /*
        botPose = limelight.getLatestResult().getBotpose();
        x = MtoIN*botPose.getPosition().x;
        y = MtoIN*botPose.getPosition().y;
        ptFinder.tryAgain(x, y, (x+10), (y) );
        telemetry.addData("position: ", x+", "+y); telemetry.addData("slope: ", ptFinder.exSlope); telemetry.addData("vectors: ", ptFinder.exVectorX+", "+ptFinder.exVectorY);
        telemetry.update();
        robot.encoderFieldCentric(0, 10, 0);


         */
        runtime.reset();

        sleep(1000);

        telemetry.addData("Done!","");
    }
}