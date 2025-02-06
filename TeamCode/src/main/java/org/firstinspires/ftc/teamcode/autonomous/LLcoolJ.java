package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PathfinderSoftware;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name= "LL", group ="LL")
public class LLcoolJ extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();
    PathfinderSoftware.pathFinder ptFinder = new PathfinderSoftware.pathFinder(this);

    private Limelight3A limelight;



    public void score(){
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PARALLEL + robot.angleConvert(15))); // score/ hook on
        while (robot.elbowDrive.isBusy()){
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if (robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(47.5))){
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(70)))){
                robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
                robot.extensionDrive.setTargetPosition(0);
                break;
            }
        }
        robot.extensionDrive.setTargetPosition(0);
        robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
    }
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        final double MtoIN = 39.3701;
        double x = 0;
        double y = 0;
        Pose3D botPose;
        double BrungX = -6.7; // FIXME find this out
        double BrungY = 29; // FIXME find this out
        double BozX = -35.8; // FIXME find this out
        double BozY = 43.4; // FIXME find this out

        double RrungX = 9.7; // FIXME find this out
        double RrungY = -27.7; // FIXME find this out
        double RozX = 36.2; // FIXME find this out
        double RozY = -44.9; // FIXME find this out

        double actingOzX;
        double actingOzY;
        double actingRungX;
        double actingRungY;

        double heading;

        robot.init(true);
        ptFinder.init(false);

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

        robot.extensionDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionDrive.setPower(0.8);

        limelight.start();
        robot.calibrateClaw(robot.ELBOW_PARALLEL);
        robot.clawYaw.setPosition(robot.YAW_MID);
        robot.clawPinch.setPosition(robot.CLAW_CLOSE);
        waitForStart();



        robot.driveFieldCentric(0.6,0,0);
        robot.encoderFieldCentric(21.25,0,0);
        sleep(100);
        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            if ((robot.elbowDrive.getCurrentPosition() >= (int) (robot.ELBOW_PARALLEL))){
                robot.extensionDrive.setTargetPosition((int) ((robot.EXTENSION_MAXIMUM_COUNT - robot.EXTENSION_COUNTS_PER_REV*4)));
            }
        }

        while (robot.elbowDrive.isBusy()){
            telemetry.addData("raising","");
        }

        score();

        if (limelight.getLatestResult() != null && limelight.getLatestResult().getBotpose() != null ){
            botPose = limelight.getLatestResult().getBotpose();
             x = botPose.getPosition().x*MtoIN;
            y = botPose.getPosition().y*MtoIN;

                if ( (x < 0) && (y > 0) ){
                    actingOzX = BozX;
                    actingOzY = BozY;
                    actingRungX = BrungX;
                    actingRungY = BrungY;
                } else {
                    actingOzX = RozX;
                    actingOzY = RozY;
                    actingRungX = RrungX;
                    actingRungY = RrungY;
                }

        } else {
            actingOzX = BozX;
            actingOzY = BozY;
            actingRungX = BrungX;
            actingRungY = BrungY;
        }
        sleep(300);

        robot.driveFieldCentric(-0.4,0,0);
        robot.encoderFieldCentric(-3.1,0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.clawAxial.setPosition(robot.CLAW_DOWN);

        // drive to pos
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_COLLAPSED));
        ptFinder.tryAgain(0,0,(46.905),(8));
        robot.encoderFieldCentric(8, 46.895,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            robot.clawPinch.setPosition(robot.CLAW_OPEN);
        }
        robot.clawAxial.setPosition(robot.CLAW_DOWN);

        // angle of claw
        robot.clawYaw.setPosition(robot.YAW_MID);

        // begin pick-up/drop sequence --> (3x)
        for (byte i=0; i<2; i++) {
            robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_COLLAPSED));
            while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
                telemetry.addData("Busy","");
                telemetry.update();
            }
            robot.clawAxial.setPosition(robot.CLAW_DOWN);
            heading = -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            robot.driveFieldCentric(0, 0, (Math.abs(heading)/heading));
            robot.encoderFieldCentric(0, 0, Math.round(heading));
            while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
                telemetry.addData("Busy","");
                telemetry.update();
            }
            while (robot.elbowDrive.isBusy()) {
                robot.clawAxial.setPosition(robot.CLAW_DOWN);
                telemetry.addData("waiting on elbow", "");
                telemetry.update();
            }
            // align claw with arm
            robot.clawAxial.setPosition(robot.CLAW_DOWN);
            sleep(100);
            // close claw / grip sample
            robot.clawPinch.setPosition(robot.CLAW_CLOSE);
            sleep(100);

            // drive back & strafe to next sample (x)
            if (i==1){
                robot.driveFieldCentric(-0.5,0,0);
                robot.encoderFieldCentric(-11.25, 0,0);
                break;
            } else {
                robot.driveFieldCentric(-0.5, 0.6, 0);
                robot.encoderFieldCentric(-11.25, 12.6, 0);
            }
            // bring arm back
            robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_BACKWARD_PARALLEL));

            // rotate claw
            robot.clawAxial.setPosition(robot.CLAW_MID);
            while (robot.elbowDrive.isBusy()){
                telemetry.addData("waiting on elbow", "");
                telemetry.update();
            }

            // drop sample
            robot.clawPinch.setPosition(robot.CLAW_OPEN);

            // drive forward to meet sample
            robot.driveFieldCentric(0.5,0,0);
            robot.encoderFieldCentric(11.25, 0, 0);
            sleep(300);
        }
        // repeat ^

        // get arm in position
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_BACKWARD_PARALLEL - robot.angleConvert(5)));
        robot.clawAxial.setPosition(robot.CLAW_MID);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy() || robot.elbowDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.driveFieldCentric(-0.6, 0, 0);
        robot.encoderFieldCentric(-4.4, 0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.clawYaw.setPosition(robot.YAW_MID);
        robot.clawPinch.setPosition(robot.CLAW_OPEN);

        heading = -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        robot.driveFieldCentric(0, 0, (Math.abs(heading)/heading));
        robot.encoderFieldCentric(0, 0, Math.round(heading));
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.driveFieldCentric(-0.4, 0, 0);
        robot.encoderFieldCentric(-6.4, 0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        // pick up specimen
        robot.clawPinch.setPosition(robot.CLAW_CLOSE);

        // begin 1st specimen scoring off sidewall
        sleep(300);

        // drive to rung
        ptFinder.tryAgain(0, 0, (-65), (19));
        robot.encoderFieldCentric(19, -65, 0);
        // score here
        robot.elbowDrive.setTargetPosition( (int) (robot.angleConvert(103))); // get arm ready
        robot.clawYaw.setPosition(robot.YAW_RIGHT);
        robot.clawAxial.setPosition(robot.CLAW_MID);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }


        while (robot.elbowDrive.isBusy()){
            telemetry.addData("raising","");
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }

        heading = -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        robot.driveFieldCentric(0, 0, (Math.abs(heading)/heading));
        robot.encoderFieldCentric(0, 0, Math.round(heading));
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.driveFieldCentric(0.4, 0, 0);
        robot.encoderFieldCentric(1.5, 0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        robot.elbowDrive.setTargetPosition((int) (robot.angleConvert(85))); // score/ hook on
        while (robot.elbowDrive.isBusy()){
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if (robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(47.5))){
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(70)))){
                robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
                robot.extensionDrive.setTargetPosition(0);
                break;
            }
        }
        robot.extensionDrive.setTargetPosition(0);
        robot.setClawPosition(robot.disable, robot.pass, robot.superposition);

        sleep(1000);

        if (limelight.getLatestResult() != null) {
            botPose = limelight.getLatestResult().getBotpose();
            x = botPose.getPosition().x * MtoIN;
            y = botPose.getPosition().y * MtoIN;
        }
        sleep(300);

        // return to OZ
        ptFinder.tryAgain(x, y, actingOzX, actingOzY);
        robot.encoderFieldCentric((actingOzX - x), (actingOzY - y), 0);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_BACKWARD_PARALLEL - robot.angleConvert(5)));
        robot.clawPinch.setPosition(robot.CLAW_OPEN);
        robot.clawYaw.setPosition(robot.CLAW_MID);
        robot.clawAxial.setPosition(robot.CLAW_MID);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        while (robot.elbowDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }

        // drive to meet
        robot.driveFieldCentric(-0.2,0,0);
        robot.encoderFieldCentric(-2,0,0);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        robot.clawPinch.setPosition(robot.CLAW_CLOSE);
        sleep(100);

        // go back to score
        ptFinder.tryAgain(actingOzX, actingOzY, actingRungX, actingRungY);
        robot.encoderFieldCentric(actingRungX-actingOzX, actingRungY-actingOzY, 0);
        // score
        robot.elbowDrive.setTargetPosition( (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(15))); // get arm ready
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if ((robot.elbowDrive.getCurrentPosition() >= (int) (robot.ARM_COUNTS_PER_DEGREE*80))){
                robot.extensionDrive.setTargetPosition((int) ((robot.EXTENSION_MAXIMUM_COUNT - robot.EXTENSION_COUNTS_PER_REV*4)));
            }
        }
        robot.clawAxial.setPosition(robot.CLAW_MID);

        while (robot.elbowDrive.isBusy()){
            telemetry.addData("raising","");
            robot.clawAxial.setPosition(robot.CLAW_MID);
        }

        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_PARALLEL + robot.angleConvert(10))); // score/ hook on
        while (robot.elbowDrive.isBusy()){
            robot.clawAxial.setPosition(robot.CLAW_MID);
            if (robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(45))){
                robot.calibrateClaw(robot.ELBOW_PERPENDICULAR);
            }
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(75)))){
                robot.setClawPosition(robot.disable, robot.pass, robot.superposition);
                robot.extensionDrive.setTargetPosition(0);
                break;
            }
        }


        if (limelight.getLatestResult() == null) {
            x = BrungX; y = BrungY;
        } else {
            botPose = limelight.getLatestResult().getBotpose();
            x = botPose.getPosition().x;
            y = botPose.getPosition().y;
        }
        while (robot.elbowDrive.isBusy()){
            robot.calibrateClaw(robot.ELBOW_PARALLEL);
            if ((robot.elbowDrive.getCurrentPosition() <= (int) (robot.ELBOW_PERPENDICULAR - robot.angleConvert(60)))){
                robot.setClawPosition(robot.disable, robot.pass, robot.pass);
                robot.clawAxial.setPosition(robot.CLAW_MID);
                robot.extensionDrive.setTargetPosition((int) (2*robot.EXTENSION_COUNTS_PER_REV));
                break;
            }
        }

        // return to OZ (finish for now)
        ptFinder.tryAgain(x, y, actingOzX, actingOzY);
        robot.encoderFieldCentric((actingOzX - x), (actingOzY - y), 0);
        sleep(300);
        robot.clawPinch.setPosition(robot.CLAW_OPEN);
        robot.clawYaw.setPosition(robot.CLAW_DOWN);
        robot.clawAxial.setPosition(robot.CLAW_MID);
        sleep(300);
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_COLLAPSED);
        while (robot.leftFrontDrive.isBusy() || robot.leftBackDrive.isBusy() || robot.rightFrontDrive.isBusy() || robot.rightBackDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        while (robot.elbowDrive.isBusy()){
            telemetry.addData("Busy","");
            telemetry.update();
        }
        // begin scoring sequence -> (after picking up all 3 samples)



        sleep(1000);

        telemetry.addData("Done!","");
    }
}