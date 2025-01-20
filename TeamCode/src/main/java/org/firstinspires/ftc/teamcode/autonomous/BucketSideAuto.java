package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name= "BUCKET Side Auto", group ="Robot")
public class BucketSideAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double heading;
        double secondsToScan = 0;
        // Initialize all the hardware using the hardware class.
        robot.init();
        robot.elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbowDrive.setPower(1.0);
        // Send a telemetry message to signify the robot waiting; wait for the game to start (driver presses PLAY)
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

            if (robot.elbowDrive.getCurrentPosition() <= (int) (robot.ARM_COUNTS_PER_DEGREE * 100) ) {
                break;
            }
        }
        robot.setClawPosition(robot.disable,robot.pass ,robot.pass); // let go of specimen
        telemetry.addData("GET ELBOW ANGLE", robot.elbowDrive.getCurrentPosition()/robot.ARM_COUNTS_PER_DEGREE);
        telemetry.update();

        sleep(200);

        robot.setClawPosition(robot.enable,robot.pass, robot.pass); // close to not hit bar
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_PARALLEL); // lower to position for pickup

        robot.driveFieldCentric(-0.5,0,0); // back away to not hit arm
        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_BACKWARD_PARALLEL);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.23) {
            telemetry.addData("BACKING UP", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(500);

        robot.driveFieldCentric(0,-1.0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.65) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        // score done!

        sleep(200);

        robot.driveFieldCentric(1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.driveFieldCentric(0,-1.0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();

        if (opModeIsActive() && robot.heading !=0) {
            robot.driveFieldCentric(0, 0, robot.turnDirection(0, true)); // ADJUST ANGLE
        }
        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
        while (opModeIsActive() && robot.heading != 0){
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10 ) * 10;
            if (heading == 0) { // go 20 under your target
                robot.driveFieldCentric(0,0,0);
                telemetry.addData("AT 180 DEG", "");
                break;
            }
            telemetry.addData("HEADING:", heading);
            telemetry.update();
        }

        sleep(1000);

        robot.driveFieldCentric(-1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.7) {
            telemetry.addData("DRIVING BACK", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();

        if (opModeIsActive() && robot.heading !=0) {
            robot.driveFieldCentric(0, 0, robot.turnDirection(0, true)); // ADJUST ANGLE
        }
        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
        while (opModeIsActive() && robot.heading != 0){
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10) * 10;
            if (heading == 0) { // go 20 under your target
                robot.driveFieldCentric(0,0,0);
                telemetry.addData("AT 180 DEG", "");
                break;
            }
            telemetry.addData("HEADING:", heading);
            telemetry.update();
        }

        sleep(200);

        robot.driveFieldCentric(1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.7) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_COLLAPSED));
        sleep(200);

        // strafe to sub zone

        robot.driveFieldCentric(0,-1.0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.4) {
            telemetry.addData("STRAFING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        sleep(200);

        robot.elbowDrive.setTargetPosition((int) robot.ELBOW_COLLAPSED);

        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();

        if (opModeIsActive() && robot.heading !=0) {
            robot.driveFieldCentric(0, 0, robot.turnDirection(0, true) ); // ADJUST ANGLE
        }
        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
        while (opModeIsActive() && robot.heading != 0){
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10 ) * 10;
            if (heading == 0) { // go 20 under your target
                robot.driveFieldCentric(0,0,0);
                telemetry.addData("AT 180 DEG", "");
                break;
            }
            telemetry.addData("HEADING:", heading);
            telemetry.update();
        }

        sleep(5000);

        robot.driveFieldCentric(-1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.7) {
            telemetry.addData("DRIVING BACK", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();

        if (opModeIsActive() && robot.heading !=0) {
            robot.driveFieldCentric(0, 0, robot.turnDirection(0, true)); // ADJUST ANGLE
        }
        robot.heading = robot.imu.getRobotYawPitchRollAngles().getYaw();
        while (opModeIsActive() && robot.heading != 0){
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/10) * 10;
            if (heading == 0) { // go 20 under your target
                robot.driveFieldCentric(0,0,0);
                telemetry.addData("AT 180 DEG", "");
                break;
            }
            telemetry.addData("HEADING:", heading);
            telemetry.update();
        }

        sleep(200);

        robot.driveFieldCentric(1.0,0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.7) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);
        robot.elbowDrive.setTargetPosition((int) (robot.ELBOW_COLLAPSED));

        sleep(200);

        robot.driveFieldCentric(0,1.0,0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.8) {
            telemetry.addData("DRIVING", "...");
            telemetry.update();
        }
        robot.driveFieldCentric(0,0,0);

        telemetry.addData("DONE","!!");
    }
}
