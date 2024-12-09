package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class VisionSoftware extends RobotHardware{

    public VisionSoftware(LinearOpMode opmode) {super(opmode);}


    public static class colorDetector extends VisionSoftware{
        public colorDetector(LinearOpMode opmode) {super(opmode);}

        // init vision variables

        public ColorBlobLocatorProcessor colorLocator;
        public List<ColorBlobLocatorProcessor.Blob> blobS;

        public ColorBlobLocatorProcessor secondaryColorLocator;
        public List<ColorBlobLocatorProcessor.Blob> secondaryBlobS;
        public VisionPortal portal;


        /**
         *
         * @param leftUp Top Left Point of the ROI you wish to set
         * @param rightDown Bottom Right Point of the ROI you wish to set
         *                  These Points can only make shapes with all perpendicular angles (only squares/rectangles)
         * @param blobList Pass the list (blob list) you wish to filter
         */
        public void filterBySetROI(Point leftUp, Point rightDown, List<ColorBlobLocatorProcessor.Blob> blobList) {
            ArrayList<ColorBlobLocatorProcessor.Blob> toRemove = new ArrayList<>();

            Point leftDown = new Point(leftUp.x, rightDown.y);
            Point rightUp = new Point(rightDown.x, leftUp.y);

            Point[] pointsSet = {leftUp, leftDown, rightUp, rightDown};

            Imgproc.minAreaRect( new MatOfPoint2f(pointsSet));

            for(ColorBlobLocatorProcessor.Blob b : blobList)
            {
                double bCenterX = b.getBoxFit().center.x;
                double bCenterY = b.getBoxFit().center.y;

                if (bCenterX <= leftUp.x || bCenterY >= leftUp.y ||
                        bCenterX >= rightDown.x || bCenterY <= rightDown.y)
                {
                    toRemove.add(b);
                }
            }

            blobList.removeAll(toRemove);
        }

        /**
         *
         * @param color What color you want (IN STRING VALUE), BLUE, RED, or YELLOW
         * @param portalQ If you want to reset the {@link VisionPortal} or not, true is yes, false is no
         * @param locatorVersion Which locator you want to set up. ONLY 2 EXIST
         * @param left How far left from the center the border should be, range of 1,-1
         * @param top How far up from the center the border should be, range of 1,-1
         * @param right How far right from the center the border should be, range of 1,-1
         * @param bottom How far down from the center the border should be, range of 1,-1
         */
        public void visionInit (String color, boolean portalQ, ColorBlobLocatorProcessor locatorVersion,
                                double left, double top, double right, double bottom) {

            switch (color) { // CUTTING EDGE CODE!!!!
                case "BLUE":
                    locatorVersion = new ColorBlobLocatorProcessor.Builder()
                            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))  // search central 1/4 of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(5)                               // Smooth the transitions between different colors in image
                            .build();
                    break;
                case "RED":
                    locatorVersion = new ColorBlobLocatorProcessor.Builder()
                            .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))  // search central 1/4 of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(5)                               // Smooth the transitions between different colors in image
                            .build();
                    break;
                case "YELLOW":
                    locatorVersion = new ColorBlobLocatorProcessor.Builder()
                            .setRoiColor(Color.rgb(0,255,0))
                            .setBoxFitColor(Color.rgb(0,0,0))
                            .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))  // search central 1/4 of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(5)                               // Smooth the transitions between different colors in image
                            .build();
                    break;
            }

            if (portalQ) {
                VisionPortal portal = new VisionPortal.Builder()
                        .addProcessor(locatorVersion)
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .setCameraResolution(new Size(1920, 1080))
                        .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .build();
                myOpMode.telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
                myOpMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
            }

        }

        @SuppressLint("DefaultLocale")
        public void detectR (Point topLeft, Point bottomRight, String locator) {
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

            switch (locator) {
                case "PRIMARY":
                    myOpMode.telemetry.addData("SCANNING", "...\n" +
                            " PRIMARY CAMERA PORTAL ACTIVE");

                    blobS = colorLocator.getBlobs(); // set list to whatever the camera found

                    filterBySetROI(topLeft, bottomRight, blobS);

                    ColorBlobLocatorProcessor.Util.filterByArea(600, 20000, blobS);  // filter out very small blobs.
                    myOpMode.telemetry.addLine(" Area Density Aspect  Center");

                    // Display the size (area) and center location for each Blob.
                    for(ColorBlobLocatorProcessor.Blob b : blobS) // telemetry the blobs found
                    {
                        RotatedRect boxFit = b.getBoxFit();
                        myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                                b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                    }
                    myOpMode.telemetry.update();
                    myOpMode.sleep(50);
                    break;
                case "BOTH": // This case will fall through into the secondary. The code here is identical to PRIMARY's
                    myOpMode.telemetry.addData("SCANNING", "...\n" +
                            " PRIMARY CAMERA PORTAL ACTIVE");

                    blobS = colorLocator.getBlobs(); // set list to whatever the camera found

                    filterBySetROI(topLeft, bottomRight, blobS); // filter by area of interest

                    ColorBlobLocatorProcessor.Util.filterByArea(600, 20000, blobS);  // filter out very small blobs.
                    myOpMode.telemetry.addLine(" Area Density Aspect  Center");

                    // Display the size (area) and center location for each Blob.
                    for(ColorBlobLocatorProcessor.Blob b : blobS) // telemetry the blobs found
                    {
                        RotatedRect boxFit = b.getBoxFit();
                        myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                                b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                    }
                case "SECONDARY":
                    myOpMode.telemetry.addData("SCANNING", "...\n" +
                            " SECONDARY CAMERA PORTAL ACTIVE");

                    secondaryBlobS = secondaryColorLocator.getBlobs(); // set list to whatever the camera found

                    filterBySetROI(topLeft, bottomRight, secondaryBlobS); // filter by area of interest

                    ColorBlobLocatorProcessor.Util.filterByArea(600, 20000, secondaryBlobS);  // filter out very small blobs.

                    // Display the size (area) and center location for each Blob.
                    for(ColorBlobLocatorProcessor.Blob b : secondaryBlobS) // telemetry the blobs found
                    {
                        RotatedRect boxFit = b.getBoxFit();
                        myOpMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                                b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                    }
                    break;

            }
        }
    }
}
