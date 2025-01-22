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
        public double clipVX;
        public double clipVY;

        /**
         * @param x1 current x position
         * @param y1 current y position
         * @param x2 target x position
         * @param y2 target y position
         */
        public void linearEncoderMovement(double x1, double y1, double x2, double y2, double minPower, float percentLoss){
            double yDifference = y2-y1;
            double xDifference = x2-x1;
            double slope;
            double Rslope;
            double xVector;
            double yVector;

            // check for undefined slope!!!
            if ((int) xDifference == 0){
                // don't move x
                xVector = 0;
                // set y to max
                yVector = (yDifference/Math.abs(yDifference));
                // set slope to communicate undefined
                slope = 404;
            } else {
                // default to this
                Rslope = xDifference/yDifference;
                slope = yDifference/xDifference;
                xVector = -( ( Rslope * Math.abs(slope)) * (xDifference/Math.abs(xDifference)));
                yVector = -( Math.abs(slope) * (yDifference/Math.abs(yDifference)));
            }
            exSlope = slope;

            // slowwww for when close
            if (Math.abs(xDifference) <= 25){
                float maxDisX = (float) (Range.clip(Math.abs(xDifference), 1, 1000)*percentLoss);
                xVector = Range.clip( xVector*maxDisX, xVector*percentLoss, 1);
                myOpMode.telemetry.addData("X SLOWED", "");
                clipVX = xVector;
                xVector = -Range.clip(Math.abs(xVector), minPower, 1);xVector *= (xDifference/Math.abs(xDifference));
            }
            if (Math.abs(yDifference) <= 25){
                float maxDisY = (float) (Range.clip(Math.abs(yDifference), 1, 1000)*percentLoss);
                yVector = Range.clip( yVector*maxDisY , yVector*percentLoss, 1);
                myOpMode.telemetry.addData("Y SLOWED","");
                clipVY = yVector;
                yVector = -Range.clip(Math.abs(yVector), minPower, 1);yVector *= (yDifference/Math.abs(yDifference));
            }

            driveFieldCentric(yVector,xVector, turnPower);
             // telemetry
            exVectorX = xVector;
            exVectorY = yVector;
            myOpMode.telemetry.addData("slope :", exSlope);
        }

        public void precise(double x1, double y1, double x2, double y2){
            double yDifference = y2-y1;
            double xDifference = x2-x1;
            double slope;
            double Rslope;
            double yVector;
            double xVector;

            if ( xDifference != 0 && yDifference != 0){
                slope = yDifference/xDifference;
                Rslope = xDifference/yDifference;
                yVector = slope;
                xVector = slope * Rslope;
            } else if ( xDifference == 0) {
                slope = 404;
                xVector = 0;
                yVector = 1;
            } else {
                slope = 0;
                xVector = 1;
                yVector = 0;
            }

            if (Math.abs(xDifference) <= 25 && xDifference != 0){
                xVector = xVector * ((Math.abs(xDifference)*0.06));
                xVector = Range.clip(Math.abs(xVector), 0.06, 1);
            }
            if (Math.abs(yDifference) <= 25 && yDifference != 0){
                yVector = yVector * ((Math.abs(yDifference)*0.06));
                yVector = Range.clip(Math.abs(yVector), 0.06, 1);
            } else if (yDifference == 0) {
                yVector = 0;
            }

            xVector = Math.abs(xVector) * -(xDifference/Math.abs(xDifference));
            yVector = Math.abs(yVector) * -(yDifference/Math.abs(yDifference));

            driveFieldCentric(yVector, xVector, turnPower);
            exVectorX = xVector;
            exVectorY = yVector;
            exSlope = slope;
        }

        public void tryAgain(double x1, double y1, double x2, double y2){
            double xDifference = x2-x1;
            double yDifference = y2-y1;
            double slope = 0;
            double denominator = Math.max(Math.abs(xDifference), Math.abs(yDifference));
            double xVector;
            double yVector;

            if ( xDifference != 0 && yDifference != 0){
                xVector = Math.round((xDifference/denominator)*1000)/1000.;
                yVector = Math.round((yDifference/denominator)*1000)/1000.;

                if (Math.abs(xDifference) < 26){ xVector *= (Math.abs(xDifference)*0.06); }
                if (Math.abs(yDifference) < 26){ yVector *= (Math.abs(yDifference)*0.06); }

                xVector = Range.clip(Math.abs(xVector), 0.06, 1);
                yVector = Range.clip(Math.abs(yVector), 0.06, 1);

                xVector *= (xDifference/Math.abs(xDifference));
                yVector *= (yDifference/Math.abs(yDifference));

            } else if ( xDifference == 0) {
                slope = 404;
                xVector = 0;
                yVector = 1;

                if (Math.abs(yDifference) < 26){ yVector *= (Math.abs(yDifference)*0.06); }
                yVector = Range.clip(Math.abs(yVector), 0.06, 1);
                yVector *= (yDifference/Math.abs(yDifference));
            } else {
                slope = 0;
                xVector = 1;

                if (Math.abs(xDifference) < 26){ xVector *= (Math.abs(xDifference)*0.06); }
                xVector = Range.clip(Math.abs(xVector), 0.06, 1);
                xVector *= (xDifference/Math.abs(xDifference));
                
                yVector = 0;
            }

            driveFieldCentric(yVector, xVector, turnPower);
            exSlope = slope;
            exVectorX = xVector; exVectorY = yVector;
            
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
