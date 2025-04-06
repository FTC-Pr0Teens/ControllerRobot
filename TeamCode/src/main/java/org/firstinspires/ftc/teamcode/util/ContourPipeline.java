package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat output = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private Rect largestRect = null;
    private double largestArea = 0;
    private long processTime = 0;

    @Override
    public Mat processFrame(Mat input) {
        long startTime = System.currentTimeMillis();

        // Convert input to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Define yellow range in HSV
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Threshold the image
        Core.inRange(hsv, lowerYellow, upperYellow, mask);

        // Find contours
        contours.clear();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Copy input to output for drawing
        input.copyTo(output);

        // Find the largest contour
        largestArea = 0;
        largestRect = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestArea) {
                Rect rect = Imgproc.boundingRect(contour);
                largestArea = area;
                largestRect = rect;
            }
        }

        // Draw the rectangle if found
        if (largestRect != null) {
            Imgproc.rectangle(output, largestRect, new Scalar(255, 0, 0), 2); // Blue rectangle
        }

        processTime = System.currentTimeMillis() - startTime;

        return output;
    }

    public boolean poleDetected() {
        return largestRect != null;
    }

    public synchronized List<Double> getContourAreas() {
        List<Double> areas = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            areas.add(Imgproc.contourArea(contour));
        }
        return areas;
    }


    public double largestContourArea() {
        return largestArea;
    }

    public Point largestContourCenter() {
        if (largestRect == null) return new Point(0, 0);
        return new Point(
                largestRect.x + largestRect.width / 2.0,
                largestRect.y + largestRect.height / 2.0
        );
    }

    public long getProcessTime() {
        return processTime;
    }
}
