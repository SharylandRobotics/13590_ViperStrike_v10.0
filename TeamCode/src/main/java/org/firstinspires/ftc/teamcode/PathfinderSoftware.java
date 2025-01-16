package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
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
        public double exVectorX;
        public double exVectorY;

        /**
         * @param x1 current x position
         * @param y1 current y position
         * @param x2 target x position
         * @param y2 target y position
         */
        public void linearEncoderMovement(double x1, double y1, double x2, double y2){
            double yDifference = y2-y1;
            double xDifference = x2-x1;
            double slope;
            double xVector;
            double yVector;

            // check for undefined slope!!!
            if (xDifference == 0){
                // don't move x
                xVector = 0;
                // set y to max
                yVector = (yDifference/Math.abs(yDifference));
                // set slope to communicate undefined
                slope = 404;
            } else {
                // default to this
                slope = yDifference/xDifference;
                xVector = ( ( 1 - Math.abs(slope)) * (xDifference/Math.abs(xDifference)));
                yVector = ( Math.abs(slope) * (yDifference/Math.abs(yDifference)));
            }
            exSlope = slope;

            exVectorX = -xVector*0.12;
            exVectorY = -yVector*0.12;

            // slowwww for when close
            if (Math.abs(xDifference) <= 2){
                driveFieldCentric(-yVector, -xVector*0.15, turnPower);
                myOpMode.telemetry.addData("X SLOWED", "");
            }
            if (Math.abs(yDifference) <= 2){
                driveFieldCentric(-yVector*0.15, -xVector, turnPower);
                myOpMode.telemetry.addData("Y SLOWED","");
            } else {
                // default
                driveFieldCentric(-yVector,-xVector, turnPower);
            } // telemetry
            myOpMode.telemetry.addData("slope :", exSlope);
        }

        public void bearingCorrection(double bearing) {
            // check if u got a readable bearing (duh)
            if (bearing != 404) {
                // turn if angle is greater than (insert #)
                if (bearing > 15) {
                    // turns the closer you are to a 90 deg angle; 0.8 to not go crazy
                    driveFieldCentric(drivePower, strafePower, (-bearing / 90) * 0.8);
                    myOpMode.telemetry.addData("Bearing over 45 deg, correcting...", "");
                } else if (bearing < -15) {
                    driveFieldCentric(drivePower, strafePower, (bearing / 90) * 0.8);
                    myOpMode.telemetry.addData("Bearing under -45 deg, correcting...", "");
                    SoundPlayer.getInstance().startPlaying(myOpMode.hardwareMap.appContext, yellowSoundID);
                } else if (bearing <= 5 && bearing >= -5) {
                    // degrees of tolerance; noticeable difference so don't sleep on this
                    driveFieldCentric(drivePower, strafePower, 0);
                    myOpMode.telemetry.addData("Bearing corrected...", "");

                }
            } else {
                // default to this (bad)
                driveFieldCentric(drivePower, strafePower, 0);
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
            return ((Math.round(x1*10)/10 == Math.round(x2*10)/10) && (Math.round(y1*10)/10 == Math.round(y2*10)/10));
        }

    }
}
