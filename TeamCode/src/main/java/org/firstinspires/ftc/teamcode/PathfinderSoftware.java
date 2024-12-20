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

        public double solveTurn(double x1, double x2, double y1, double y2, double targetHeading) {
            // pythagorean theorem
            double distance = Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
            double deltaHeading = turnDirection(Math.round(targetHeading), false);

            return Math.abs((distance)/deltaHeading)*0.0;
        }

        /**
         *
         * @param x1 current x position
         * @param x2 target x position
         * @param y1 current y position
         * @param y2 target y position
         * @param targetHeading target heading
         */
        public void pathFind(double x1, double x2, double y1, double y2, double targetHeading){
            double yDifference = y2-y1;
            double xDifference = x2-x1;

            yDifference = Range.clip(yDifference, -1, 1);
            xDifference = Range.clip(xDifference, -1, 1);

            driveFieldCentric(yDifference*0.1,xDifference*0.1, solveTurn(x1, x2, y1, y2, targetHeading));
        }

        /**
         *
         * @param x1 current x position
         * @param x2 target x position
         * @param y1 current y position
         * @param y2 target y position
         * @return whether robot has reached the target position. (true means at target pos)
         */
        public boolean atTargetPos(double x1, double x2, double y1, double y2){
            return ((Math.round(x1) == Math.round(x2)) && (Math.round(y1) == Math.round(y2)));
        }

    }
}
