package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
AVERAGE PROCESSING TIME:
This pipeline is designed to filter out the yellow pole and replace the background with black
 */
public class YellowPipeline extends OpenCvPipeline {

    // for tracking pipeline processing speed
    private ElapsedTime timer = new ElapsedTime();
    private double processTime = 0;

    // constants
    private final Rect LEFT_RECT = new Rect(1, 1, 300, 447);
    private final Rect MID_RECT = new Rect(301, 1, 500, 447);
    private final Rect RIGHT_RECT = new Rect(501, 1, 799, 447);
    private final Size KERNEL = new Size(20, 20);
    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar UPPER_HSV = new Scalar(30, 255, 255);
    private final Scalar LOWER_HSV = new Scalar(10, 130, 130);

    // final output image
    Mat output = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        // blurring the input image to improve colour detection
        Mat blur = new Mat();
        Imgproc.blur(input, blur, KERNEL);

        // convert blurred image into HSV scale (Hue, Saturation, Brightness Value)
        Mat hsv = new Mat();
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        // create a HSV threshold that filters out yellow shades
        Mat threshold = new Mat();
        Core.inRange(hsv, LOWER_HSV, UPPER_HSV, threshold);

        // use the threshold image as a mask to filter out all the yellow in the original
        Mat masked = new Mat();
        Core.bitwise_and(hsv, hsv, masked, threshold);

        // convert the now isolated yellow pole image back into RGB
        Imgproc.cvtColor(masked, output, Imgproc.COLOR_HSV2RGB);

        // drawing three rectangles on camera feed
        Imgproc.rectangle(output, RIGHT_RECT, WHITE, 1);
        Imgproc.rectangle(output, MID_RECT, WHITE, 1);
        Imgproc.rectangle(output, LEFT_RECT, WHITE, 1);

        // deallocating matrix memory
        blur.release();
        hsv.release();
        threshold.release();
        masked.release();

        // tracking pipeline speed
        processTime = timer.milliseconds();
        timer.reset();

        return output; // what the camera stream will display on stream
    }

    public double getProcessTime() {
        return processTime;
    }
}