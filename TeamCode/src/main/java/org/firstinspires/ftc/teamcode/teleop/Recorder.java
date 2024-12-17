package org.firstinspires.ftc.teamcode.teleop;

import java.io.Writer;

//https://www.youtube.com/watch?v=OFX96ONRU18

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
@TeleOp(name = "Recorder/debug", group = "Robot")
public class Recorder extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        double heading;
        robot.init();
        waitForStart();

        while (opModeIsActive()) {

        }
    }
}