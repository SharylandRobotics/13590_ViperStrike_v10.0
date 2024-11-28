package org.firstinspires.ftc.teamcode;

// hardware & class imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// camera imports
import android.annotation.SuppressLint;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import java.util.List;


public class RobotHardware {

    // Declare OpMode members
    private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor elbowDrive = null;
    public DcMotor extensionDrive = null;
    public WebcamName myEyes = null; // CAMERA!! remember to change the type of this var if not available on Dhub
    public Servo clawPinch = null;
    private CRServo clawYaw = null;
    public Servo clawAxial = null;

    // Define Sensor objects (Make them private so that they CANT be accessed externally)
    public IMU imu = null; // Universal IMU interface

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
    public double ELBOW_SPEED;

    public double CLAW_CLOSE;
    public double CLAW_OPEN;
    public double CLAW_DOWN;
    public double CLAW_MID;
    public double CLAW_UP;
    public double CLAW_IN;
    public double CLAW_OUT;

    // End of Rudimentary inits...


    // initialize enums ; easier to read :D
    public enum statesOfBeing {
        ACTIVATE,
        DEACTIVATE,
        PASS,
        SUPERPOSITION
    }

    // initialize enum constants ; this is for passing enum values into other classes
    public statesOfBeing enable = statesOfBeing.ACTIVATE;
    public statesOfBeing disable = statesOfBeing.DEACTIVATE;
    public statesOfBeing superposition = statesOfBeing.SUPERPOSITION;
    public statesOfBeing pass = statesOfBeing.PASS;

    // Declare Elbow Encoder Variables, REMEMBER TO DECLARE WHEEL ONES LATER!!
    public final double COUNTS_PER_DEGREE =
        28 // counts for motor revolution...
            * (250047.0 / 4913.0) // times internal gearing (yes, counts per motor rev are the BARE drive)
            * ((double) 100 / 20) // external gearing, 20 to 100 teeth
            * ((double) 1 /360); // ... per degree
    public final int ELBOW_ANGLE_OFFSET = 59;

    /* Elbow Positions
     ELBOW_ANGLE_OFFSET is the offset angle the arm starts off on
     the numbers added after it are tweaks for more accurate positions due to gravity and such
     Rounding converts the value into a long (dk why) so it shows up like 90.0, add (int) to convert

     VARIABLES CONCERNING ANGLES HAVE THIS RULE APPLIED:

     TRUE angles are angles RELATIVE to the MOTOR STARTING POINT, NOT TO YOUR PREFERRED PLANE OF REFERENCE
     STANDARD angles, or angles w/o the TRUE suffix are angles BASED on the PLANE OF REFERENCE

     SN: these variables declared here are the only ones that are exempt to this rule as they are used when stating
         positions in the executive code and i do not wish to cause confusion there.
     */
    public double ELBOW_COLLAPSED = 0;
    public double ELBOW_PARALLEL = Math.round((ELBOW_ANGLE_OFFSET) * COUNTS_PER_DEGREE);
    public double ELBOW_ANGLED = Math.round((45 + ELBOW_ANGLE_OFFSET) * COUNTS_PER_DEGREE);
    public double ELBOW_PERPENDICULAR = Math.round((90 + ELBOW_ANGLE_OFFSET) * COUNTS_PER_DEGREE);
    public double ELBOW_ANTI_ANGLED = Math.round((135 + ELBOW_ANGLE_OFFSET) * COUNTS_PER_DEGREE);
    public double ELBOW_ANTI_PARALLEL = Math.round((180 + ELBOW_ANGLE_OFFSET) * COUNTS_PER_DEGREE);
    public double ELBOW_ANTI_COLLAPSED = Math.round((225 + ELBOW_ANGLE_OFFSET) * COUNTS_PER_DEGREE);
    public double ELBOW_FUDGE_FACTOR = 5 * COUNTS_PER_DEGREE; // Amount to rotate the elbow by

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
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class,"left_front_drive"); // Stuff on CH
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        elbowDrive = myOpMode.hardwareMap.get(DcMotor.class, "elbow_drive"); // Stuff on EH
        extensionDrive = myOpMode.hardwareMap.get(DcMotor.class, "extension_drive");
        clawPinch = myOpMode.hardwareMap.get(Servo.class, "claw_pinch");
        clawYaw = myOpMode.hardwareMap.get(CRServo.class, "claw_yaw");
        clawAxial = myOpMode.hardwareMap.get(Servo.class, "claw_axial");

        myEyes = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        DRIVE_SPEED = 0.5; // Maximum autonomous driving speed for better distance accuracy.
        STRAFE_SPEED = 0.5; // Maximum autonomous strafing speed for better distance accuracy.
        TURN_SPEED = 0.4; // Maximum autonomous turning speed for better rotational accuracy.
        ELBOW_SPEED = 1.0; // Maximum elbow speed.

        CLAW_CLOSE = 0.4; // TBD
        CLAW_OPEN = 0.1; // TBD

        CLAW_DOWN = 0.92; // MAXIMUM, not playable position
        CLAW_MID = 0.51; // playable position
        CLAW_UP = 0.08; // MAXIMUM, not playable position

        /*
            Playable positions, initialize them if you feel the need to do so in the future:
            CLAW_BOTTOM = 0.3; // Claw at collapsed position
            CLAW_ANGLED = 0.67; // Claw at angled position
            CLAW_ANTI_BOTTOM = 0.72 // Claw at anti collapsed position
            Not initialized because you can just call the calibrateClaw(); function
         */

        CLAW_IN = 0.0; // TBD
        CLAW_OUT = 0.6; // TBD

        // Define and initialize ALL installed sensors (note: need to use reference to the actual OpMode).
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

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

        elbowDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ensure elbow starts at 0
        elbowDrive.setTargetPosition(0);
        elbowDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the IMU when initializing the hardware class
        imu.resetYaw();

        // Wait for the game to start (Display Gyro value while waiting)
        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.addData("Wheels starting at", "%7d :%7d :%7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.addData("Starting Elbow Pos:", elbowDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }
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



    public void setClawPosition(statesOfBeing pinch, float yaw, statesOfBeing axial, double extension) {

        //noinspection StatementWithEmptyBody
        if (pinch == pass){ // here to make sure it doesn't waste processing power

        } else if (pinch == enable) { clawPinch.setPosition(CLAW_CLOSE); // Closes Claw:

        } else if (pinch == disable) { clawPinch.setPosition(CLAW_OPEN); } // Opens Claw:


        //noinspection StatementWithEmptyBody
        if (axial == pass) { // here to make sure it doesn't waste processing power

        } else if (axial == enable) { clawAxial.setPosition(CLAW_UP); // Raises Claw:

        } else if (axial == disable) { clawAxial.setPosition(CLAW_DOWN); // Lowers Claw:

        } else if (axial == superposition) { clawAxial.setPosition(CLAW_MID); // Parallels Claw to floor:

        }

        clawYaw.setPower(yaw); // Rotates the claw based on trigger value

        extensionDrive.setPower(extension); // ** Will Change to be like clawYaw **
    }
    // to stop the claw from switching back and forth while in perpendicular position
        /* more specifically, it stops the elbow from having a preferred side to turn to when at 149 deg
            for ex. Without this, if you went to the perpendicular position from the backwards side,
            it would automatically face the claw the other way
         */
    statesOfBeing elbowDirection = enable;
    public void calibrateClaw(double orientation) { // VERY IMPORTANT: moves the claw to face parallel/perpendicular to the ground
                                                    // relative to the elbow's position
        // returns the current angle without the offset
        double elbowDegTRUE = Math.round((elbowDrive.getCurrentPosition() / COUNTS_PER_DEGREE));
        double targetClawPos; // used to avoid changing the claw position too many times in here

        if (elbowDegTRUE < 149) { // sets function to normal
            targetClawPos = ((-0.17 / 45) * Math.abs(elbowDegTRUE - 149) + 0.85); // this if for when the elbow is normal
            elbowDirection = enable;
        } else if (elbowDegTRUE > 149){ // changes function to inverse
            targetClawPos = ((0.16 / 45) * Math.abs(elbowDegTRUE - 149) + 0.19); // this is for when the elbow is backwards
            elbowDirection = disable;
        } else { // for increased accuracy, set to 90 base floor
            if (elbowDirection == enable) {
                targetClawPos = (0.85);
            } else {
                targetClawPos = (0.19);
            }
        }

        if (orientation == ELBOW_PERPENDICULAR){ // change position to perpendicular to floor
            if (elbowDegTRUE < 84 && elbowDegTRUE > 27) { // sets limit between 84 deg from collapsed and 27 deg from collapsed
                targetClawPos = ((-0.17 / 45) * Math.abs(elbowDegTRUE - 84) + CLAW_DOWN); // this if for when the elbow is normal
                elbowDirection = enable;
            } else if (elbowDegTRUE > 214 && elbowDegTRUE < 271){ // sets limit between 214 deg from collapsed and 271 deg from collapsed
                targetClawPos = ((0.16 / 45) * Math.abs(elbowDegTRUE - 214) + CLAW_UP); // this is for when the elbow is backwards
                elbowDirection = disable;
            }
        }

        clawAxial.setPosition(targetClawPos);
    }

    /*
    ------------------------- YOU ARE ENTERING THE CAMERA SOFTWARE ZONE --------------------------
                                just beware yk.
    ------------------------- YOU ARE ENTERING THE CAMERA SOFTWARE ZONE --------------------------
     */

    // init vision variables

    public ColorBlobLocatorProcessor colorLocator;
    public VisionPortal portal;
    public List<ColorBlobLocatorProcessor.Blob> blobs;

    public void visionInit () {
         colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        myOpMode.telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        myOpMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

    }

    @SuppressLint("DefaultLocale")
    public void detectR () {
        /* ----- USE THIS FUNC LIKE SO... -----
                                while (condition)
                                {
                                    robot.detectR();  // analyzes camera feed and puts blobs it finds into a list

                                    if (blobs (operator) (condition)) {  // blobs is the list mentioned above...
                                        (action)
                                    } // this checks if the list of blobs (condition) and if it is met it executes (action)
                                }

                                OR

                                while (true)
                                {
                                    robot.detectR();  // analyzes camera feed and puts blobs it finds into a list

                                    if (blobs (operator) (condition)) {  // blobs is the list mentioned above...
                                        break;
                                    } // this checks if the list of blobs (condition) and if it is met it breaks out the loop
                                }

                                (action); // action is then performed

         */


        myOpMode.telemetry.addData("wakey wakey...", "\n" + // init message :D
                "⠀⠀⠀⠄⠀⠀⠀⠀⠄⠂⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\n" +
                "⠀⠔⠁⠀⠀⠀⠀⠀⠀⠂⠀⠁⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠐⠂⠠⡀⠀⠀⠀\n" +
                "⠀⣠⣶⣾⣾⣿⡿⣷⡀⢐⠀⠀⠁⠀⠀⡰⠃⣀⣀⣀⠀⠀⠀⠀⠀⠀⠈⠢⠀⠀\n" +
                "⢸⣿⢻⣿⣿⣿⣷⣹⡗⢨⠀⠀⠀⡆⠠⠀⣾⢛⣿⣿⣿⣶⣦⣄⠀⠀⠀⠀⠈⠀\n" +
                "⠀⢿⣿⣿⡻⣿⣿⣿⠃⠀⠀⠀⢀⡧⠆⢠⡏⣾⣿⣿⡏⠉⣿⣿⣿⣦⣄⠀⠀⠀\n" +
                "⠀⠈⢿⣿⣿⣿⡿⠃⠀⡘⠀⢀⠔⠀⡄⠀⣷⣿⣿⣿⣷⣾⣿⣿⣿⣿⣿⣷⣄⠀\n" +
                "⠀⠀⠀⠙⢩⠀⢀⣤⠊⠔⠚⠀⠀⠀⠘⡀⠈⠿⣿⣿⣿⣿⣹⣿⣿⣿⢟⡵⠣⠂\n" +
                "⠀⠀⠀⠠⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠂⢄⠈⠙⠻⠿⣿⡿⠿⡻⠝⠀⠀⠀\n" +
                "⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠒⠀⠀⠀⠀⠘⠀⠀⠀⠀⠀\n" +
                "⠀⠀⠆⠀⠀⠀⠀⠀⠀⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠡⠀⠀⠀⠀\n" +
                "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠛⠛⠒⠒⠦⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠄⠀⠀⠀\n" +
                "⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀");

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs(); // set list to whatever the camera found

        ColorBlobLocatorProcessor.Util.filterByArea(200, 20000, blobs);  // filter out very small blobs.

        myOpMode.telemetry.addLine(" Area Density Aspect  Center");

        // Display the size (area) and center location for each Blob.
        for(ColorBlobLocatorProcessor.Blob b : blobs) // telemetry the blobs found
        {
            RotatedRect boxFit = b.getBoxFit();
            myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
        }

        myOpMode.telemetry.update();
        myOpMode.sleep(50);

    }


}