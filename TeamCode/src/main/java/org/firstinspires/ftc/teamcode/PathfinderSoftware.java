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
                xVector = -( ( 1 - Math.abs(slope)) * (xDifference/Math.abs(xDifference)));
                yVector = -( Math.abs(slope) * (yDifference/Math.abs(yDifference)));
            }
            exSlope = slope;

            // slowwww for when close
            if (Math.abs(xDifference) <= 25){
                xVector = Range.clip( xVector*( Range.clip(Math.abs(xDifference), 7, 1000)*0.01) , xVector*0.02, xVector);
                myOpMode.telemetry.addData("X SLOWED", "");
            }
            if (Math.abs(yDifference) <= 25){
                yVector = Range.clip( yVector*( Range.clip(Math.abs(yDifference), 7, 1000)*0.01) , yVector*0.02, yVector);
                myOpMode.telemetry.addData("Y SLOWED","");
            }
            driveFieldCentric(yVector,xVector, turnPower);
             // telemetry
            exVectorX = xVector;
            exVectorY = yVector;
            myOpMode.telemetry.addData("vectors: ", Math.round(xVector*100)/100 + ", " + Math.round(yVector*100)/100);
            myOpMode.telemetry.addData("slope :", exSlope);
        }

        public void bearingCorrection(double bearing) {
            // check if u got a readable bearing (duh)
            if (bearing != 404) {
                 if (!(bearing <= 5 && bearing >= -5)) {
                    // degrees of tolerance; noticeable difference so don't sleep on this
                    driveFieldCentric(drivePower, strafePower, (bearing / 60) * ((drivePower+strafePower)/2));
                 } else {driveFieldCentric(drivePower, strafePower, 0);
                    myOpMode.telemetry.addData("Bearing corrected...", "");}
            } else {
                // default to this (bad)
                driveFieldCentric(drivePower, strafePower, 0);
                myOpMode.telemetry.addData("NOT CORRECTING BEARING","");
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
