package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper bounds for the colors (HSV)
    private final Scalar
            YELLOW = new Scalar(0,255,255),
            CYAN = new Scalar(255,255,0),
            MAGENTA = new Scalar(255,0,255),
            YELLOW_LOWER = new Scalar(20, 100, 100),
            YELLOW_UPPER = new Scalar(40, 255, 255),
            CYAN_LOWER = new Scalar(75, 0, 0),
            CYAN_UPPER = new Scalar(100, 255, 255),
            MAGENTA_LOWER = new Scalar(140, 0, 0),
            MAGENTA_UPPER = new Scalar(160, 255, 255);

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.CENTER;


    @Override
    public Mat processFrame(Mat input) {
        // Convert the input image to the HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create a Mat variable named binary
        Mat binary = new Mat();

        // Get the submat frame
        Mat areaMat = hsv.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Create a mask for each color
        Core.inRange(areaMat, YELLOW_LOWER, YELLOW_UPPER, binary);

        // Apply a threshold filter to reduce noise
        Imgproc.threshold(binary, binary, 50, 255, Imgproc.THRESH_BINARY);

        // Count the number of non-zero pixels
        int yellowPixels = Core.countNonZero(binary);

        // Repeat the same process for the other colors
        Core.inRange(areaMat, CYAN_LOWER, CYAN_UPPER, binary);
        Imgproc.threshold(binary, binary, 50, 255, Imgproc.THRESH_BINARY);
        int cyanPixels = Core.countNonZero(binary);

        Core.inRange(areaMat, MAGENTA_LOWER, MAGENTA_UPPER, binary);
        Imgproc.threshold(binary, binary, 50, 255, Imgproc.THRESH_BINARY);
        int magentaPixels = Core.countNonZero(binary);

        // Determine which color has the most pixels
        if (yellowPixels > cyanPixels && yellowPixels > magentaPixels) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (cyanPixels > yellowPixels && cyanPixels > magentaPixels) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }
            // Release the mats
            hsv.release();
            binary.release();
            areaMat.release();

            // Return the input image
            return input;

    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}