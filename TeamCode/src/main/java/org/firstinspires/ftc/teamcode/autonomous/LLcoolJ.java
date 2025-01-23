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
        limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        final double MtoIN = 39.3701;
        double x = 0;
        double y = 0;
        double bearing = 0;
        Pose3D botPose;
        double xHold;
        double yHold;

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

        limelight.start();

        waitForStart();

        ptFinder.tryAgain(x, y, (x), (y+27) );
        robot.encoderFieldCentric(27,0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        sleep(500);

        botPose = limelight.getLatestResult().getBotpose();
        x = MtoIN*botPose.getPosition().x;
        y = MtoIN*botPose.getPosition().y;
        ptFinder.tryAgain(x, y, (x+10), (y) );
        telemetry.addData("position: ", x+", "+y); telemetry.addData("slope: ", ptFinder.exSlope); telemetry.addData("vectors: ", ptFinder.exVectorX+", "+ptFinder.exVectorY);
        telemetry.update();
        robot.encoderFieldCentric(0, 10, 0);

        runtime.reset();

        sleep(1000);

        telemetry.addData("Done!","");
    }
}