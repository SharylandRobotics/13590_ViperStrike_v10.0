package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    // Declare OpMode members
    private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor liftDrive = null;
    // public DcMotor extensionDrive = null;
    public WebcamName myEyes = null; // CAMERA!! remember to change the type of this var if not available on Dhub
    public Servo clawPinch = null;
    private CRServo clawYaw = null;
    public Servo clawAxial = null;
    private Servo clawExtension = null;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    private IMU imu = null; // Universal IMU interface

    /*
    These variables are declared here (as class members) so they can be updated in various methods, but still be
    displayed by sendTelemetry()
     */
    public ElapsedTime runtime = new ElapsedTime();

    // Rudimentary initialization of variables
    public double drivePower;
    public double strafePower;
    public double turnPower;

    public double leftFrontPower;
    public double leftBackPower;
    public double rightFrontPower;
    public double rightBackPower;

    public double DRIVE_SPEED;
    public double STRAFE_SPEED;
    public double TURN_SPEED;
    public double LIFT_SPEED;

    public double CLAW_CLOSE;
    public double CLAW_OPEN;
    public double CLAW_DOWN;
    public double CLAW_UP;
    public double CLAW_IN;
    public double CLAW_OUT;
    public double LIFT_TOP;
    // End of Rudimentary inits...


    // initialize enums ; easier to read :D
    public enum statesOfBeing {
        ACTIVATE,
        DEACTIVATE,
        PASS
    }

    // initialize enum constants ; this is for passing enum values into other classes
    public statesOfBeing enable = statesOfBeing.ACTIVATE;
    public statesOfBeing disable = statesOfBeing.DEACTIVATE;
    public statesOfBeing pass = statesOfBeing.PASS;


    // Declare Lift Encoder Variables, REMEMBER TO DELCARE WHEEL ONES LATER!!
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: GoBILDA 117 Motor Encoder
    static final double     SPOOL_DIAMETER_INCHES   = 1.4035433 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (SPOOL_DIAMETER_INCHES * 3.1415);


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robots' hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map and initialized.
     */
    public void init(){

        // Define and initialize ALL installed motors (note: need to use reference to the actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class,"left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        liftDrive = myOpMode.hardwareMap.get(DcMotor.class, "lift_drive");
        // extensionDrive = myOpMode.hardwareMap.get(DcMotor.class, "extension_drive");

        clawPinch = myOpMode.hardwareMap.get(Servo.class, "claw_pinch");
        clawYaw = myOpMode.hardwareMap.get(CRServo.class, "claw_yaw");
        clawAxial = myOpMode.hardwareMap.get(Servo.class, "claw_axial");
        clawExtension = myOpMode.hardwareMap.get(Servo.class, "claw_extension");

        myEyes = myOpMode.hardwareMap.get(WebcamName.class, "myEyes");

        DRIVE_SPEED = 0.5; // Maximum autonomous driving speed for better distance accuracy.
        STRAFE_SPEED = 0.5; // Maximum autonomous strafing speed for better distance accuracy.
        TURN_SPEED = 0.4; // Maximum autonomous turning speed for better rotational accuracy.
        LIFT_SPEED = 1.0; // Maximum lift speed.

        CLAW_CLOSE = 0.8; // TBD
        CLAW_OPEN = 0.0; // TBD

        CLAW_DOWN = 0.0; // TBD
        CLAW_UP = 0.6; // TBD

        CLAW_IN = 0.0; // TBD
        CLAW_OUT = 0.6; // TBD

        LIFT_TOP = 15; // TBD

        // Define and initialize ALL installed sensors (note: need to use reference to the actual OpMode).
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        /*
         Most robots need the motors on one side to be reversed to drive forward. The motor reversals shown here are
         for a "direct drive" robot (the wheels turn in the same direction as the motor shaft). If your robot has
         additional gear reductions or uses a right-angled drive, it is important to ensure that your motors are turning
         in the correct direction. So, start out with the reversals here, BUT when you first test your robot, push the
         left joystick forward and observe the wheels turn. Reverse the direction (flip FORWARD <-> REVERSE) of any
         wheel that runs backward. Keep testing until ALL the wheels move the robot forward when you push the left
         joystick forward.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        // extensionDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // extensionDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE



        // Reset the IMU when initializing the hardware class
        imu.resetYaw();

        // Wait for the game to start (Display Gyro value while waiting)
        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }
    }

    /* Stop and reset encoders ; made into a function to avoid clutter & to remove necessary declarations
        Ex. reseting encoders on auto by time or in TeleOp modes, which may affect performance
     */
    public void stopNreset() {
        /* stop and reset encoders!! ( for later )

         */

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param drive     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {

        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(driveRotation) + Math.abs(strafeRotation) + Math.abs(turn), 1);
        leftFrontPower = (driveRotation + strafeRotation + turn) / denominator;
        leftBackPower = (driveRotation - strafeRotation + turn) / denominator;
        rightFrontPower = (driveRotation - strafeRotation - turn) / denominator;
        rightBackPower = (driveRotation +  strafeRotation - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /* function for using the encoder on the liftDrive, may be able to be included in a
       master encoder function later on (a function with all encoders in it, wheels + lift)
     */

    // Following are variables to pass values across the next 2 functions defined
    private int newLiftTarget;
    private double timeoutEXTERNAL;

    // 1st half of encoderLift, used to set a distance target, NEEDS 2nd half to finish action/not break
    public void encoderLift(double speed, double inches, double timeoutS) {
        if (myOpMode.opModeIsActive()) {

            timeoutEXTERNAL = timeoutS;

            // Determine new target position, and pass to motor controller
            newLiftTarget = liftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            liftDrive.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            liftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

        }


    }

    // 2nd half of encoderLift function, used to perform actions before/while encoderLift is running
    public void encoderLiftFinish(boolean timeout){
            // Display it for the driver.
        myOpMode.telemetry.addData("Path1", "Running to %7d :%7d", newLiftTarget);
        myOpMode.telemetry.addData("Path2", "Running at %7d :%7d",
                liftDrive.getCurrentPosition());
        myOpMode.telemetry.update();
        if (!liftDrive.isBusy()) {
            liftDrive.setPower(0);

            liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (timeout) {
            myOpMode.resetRuntime();
            while ((runtime.seconds() < timeoutEXTERNAL) && liftDrive.isBusy()) {
                myOpMode.telemetry.addData("Path1", "Running to %7d :%7d", newLiftTarget);
                myOpMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        liftDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            liftDrive.setPower(0);

            liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    /**
     * Pass the requested wheel motor power to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel,
                              double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }



    public void setClawPosition(statesOfBeing pinch, double yaw, statesOfBeing axial, double extension) {

        //noinspection StatementWithEmptyBody
        if (pinch == pass){ // here to make sure it doesn't waste processing power

        } else if (pinch == enable) { clawPinch.setPosition(CLAW_CLOSE); // Closes Claw:

        } else if (pinch == disable) { clawPinch.setPosition(CLAW_OPEN); } // Opens Claw:


        //noinspection StatementWithEmptyBody
        if (axial == pass) { // here to make sure it doesn't waste processing power

        } else if (axial == enable) { clawAxial.setPosition(CLAW_UP); // Raises Claw:

        } else if (axial == disable) { clawAxial.setPosition(CLAW_DOWN); } // Lowers Claw:

        clawYaw.setPower(yaw); // Rotates the claw based on trigger value

        clawExtension.setPosition(extension); // ** Will Change to be like clawYaw **
    }

    public void setLiftPosition(statesOfBeing position) { // USE ONLY AFTER DEFINING LIFT_TOP

        if (position == pass) { // here to make sure it doesn't waste processing power
            encoderLiftFinish(false);

        } else if (position == enable) {
            encoderLift(LIFT_SPEED,Math.abs(LIFT_TOP - liftDrive.getCurrentPosition()) // this calculates the distance from
                                                                                // the current position to the top position
                    ,15);

        } else if (position == disable) {
            encoderLift(LIFT_SPEED,Math.abs(-liftDrive.getCurrentPosition()) // this calculates the distance to the bottom pos
                    ,15);
        }
    }
}