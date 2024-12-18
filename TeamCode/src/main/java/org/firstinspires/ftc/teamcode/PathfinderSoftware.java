package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Mat;

import java.util.Objects;

public class PathfinderSoftware extends RobotHardware{
    public PathfinderSoftware(LinearOpMode opmode) {super(opmode);}

    public static class pathFinder extends PathfinderSoftware{

        public pathFinder(LinearOpMode opmode) {
            super(opmode);
        }
        public double solveTurn(double x1, double x2, double y1, double y2, double targetHeading, double CurrentHeading) {
            // pythagorean theorem
            double distance = Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
            double goal = targetHeading - CurrentHeading;
            if (goal > 180) {
                goal -= 360;
            }
            if (goal < -180) {
                goal += 360;
            }



            return Math.abs(distance) / goal;
        }

        public double solveDrive(double y1, double y2) {
            return y1 / y2;
        }

        public double solveSlope(double x1, double x2, double y1, double y2){
            double slope = (y2 - y1) / (x2 - x1);
            return slope;
        }

        public double solveStrafe(double x1, double x2) {
            return x1 / x2;
        }

        /**
         *
         * @param x1 current x position
         * @param x2 target x position
         * @param y1 current y position
         * @param y2 target y position
         * @param targetHeading target heading
         */
        public void pathFind(double x1, double x2, double y1, double y2, double targetHeading, double currentHeading){
            driveFieldCentric(y2-y1,x2-x1, solveTurn(x1, x2, y1, y2, targetHeading, currentHeading));
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
