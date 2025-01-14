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

            double yVector = ( Math.abs(slope) * (yDifference/Math.abs(yDifference)));
            double xVector = ( ( 1 - Math.abs(slope)) * (xDifference/Math.abs(xDifference)));

            driveFieldCentric(-yVector*0.0,-xVector*0.0, 0);
            myOpMode.telemetry.addData("slope :", exSlope);
        }

        public void bearingCorrection(double bearing) {
            if (bearing != 404) {
                if (bearing > 15) {
                    driveFieldCentric(drivePower, strafePower, (-bearing / 90) * 0.8);
                    myOpMode.telemetry.addData("Bearing over 45 deg, correcting...", "");
                    SoundPlayer.getInstance().startPlaying(myOpMode.hardwareMap.appContext, yellowSoundID);
                } else if (bearing < -15) {
                    driveFieldCentric(drivePower, strafePower, (bearing / 90) * 0.8);
                    myOpMode.telemetry.addData("Bearing under -45 deg, correcting...", "");
                    SoundPlayer.getInstance().startPlaying(myOpMode.hardwareMap.appContext, yellowSoundID);
                } else if (bearing <= 4 && bearing >= -4) {
                    driveFieldCentric(drivePower, strafePower, 0);
                    myOpMode.telemetry.addData("Bearing corrected...", "");

                }
            } else {
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
