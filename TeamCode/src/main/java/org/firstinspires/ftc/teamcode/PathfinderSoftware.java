package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public class PathfinderSoftware extends RobotHardware{
    public PathfinderSoftware(LinearOpMode opmode) {super(opmode);}

    public static class pathFinder extends PathfinderSoftware{
        public pathFinder(LinearOpMode opmode) {
            super(opmode);
        }

        /**
         * CALL THIS FUNCTION OR ELSE IMU WILL TURN UP NULL!!!! (it breaks)
         */
        public void init() {
            super.init();
        }

        public double exSlope;
        /**
         * @param x1 current x position
         * @param y1 current y position
         * @param x2 target x position
         * @param y2 target y position
         */
        public void linearEncoderMovement(double x1, double y1, double x2, double y2){
            double yDifference = y2-y1;
            double xDifference = x2-x1;

            double slope = yDifference/xDifference;
            exSlope = slope;

            float yVector = (float) ( Math.abs(slope) * (yDifference/Math.abs(yDifference)));
            float xVector = (float) ( ( 1 - Math.abs(slope)) * (xDifference/Math.abs(xDifference)));

            driveFieldCentric(yDifference*0.1,-xDifference*0.1, 0);
        }

        public void bearingCorrection(double bearing){
            if (bearing > 45 ){
                driveFieldCentric(drivePower, strafePower, -0.5);
                myOpMode.telemetry.addData("Bearing over 45 deg, correcting...", "");
            } else if (bearing < -45) {
                driveFieldCentric(drivePower, strafePower, 0.5);
                myOpMode.telemetry.addData("Bearing under -45 deg, correcting...", "");
            }
        }

        /**
         * @param x1 current x position
         * @param y1 current y position
         * @param x2 target x position
         * @param y2 target y position
         * @return whether robot has reached the target position. (true means at target pos)
         */
        public boolean atTargetPos(double x1, double y1, double x2, double y2){
            return ((Math.round(x1) == Math.round(x2)) && (Math.round(y1) == Math.round(y2)));
        }

    }
}
