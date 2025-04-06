package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.util.ContourPipeline;
import org.firstinspires.ftc.teamcode.util.Yellowcountour;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraSubsystem {
    public OpenCvCamera camera;
    private ContourPipeline contourPipeline; // pipeline designed to look for contours of the poles
    private AprilTagPipeline aprilTagPipeline;
    private int cameraMonitorViewId; // ID of the viewport which camera feed will be displayed

    public static final int VIEW_WIDTH = 320;
    public static final int VIEW_HEIGHT = 176;
    public static final int CENTER_X = VIEW_WIDTH / 2;
    public static final int CENTER_Y = VIEW_HEIGHT / 2;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public enum PipelineName {
        CONTOUR_PIPELINE, APRILTAG_PIPELINE
    }

    public CameraSubsystem(HardwareMap hardwareMap, PipelineName pipeline){
        OpenCvPipeline selectedPipeline = aprilTagPipeline;

        // connect whichever pipeline is desired and comment the other one
        if(pipeline == PipelineName.APRILTAG_PIPELINE){
            aprilTagPipeline = new AprilTagPipeline(0.166, fy, fx, cy, cx);
            selectedPipeline = aprilTagPipeline;
        }
        else if(pipeline == PipelineName.CONTOUR_PIPELINE){
            contourPipeline = new ContourPipeline();
            selectedPipeline = contourPipeline;
        }

        // initiate the needed parameters
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // initiate the camera object with created parameters and pipeline
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(selectedPipeline);

        // runs camera on a separate thread so it can run simultaneously with everything else
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                // starts the camera stream when init is pressed
                camera.startStreaming(VIEW_WIDTH,VIEW_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public ContourPipeline getPipeline() {
        return contourPipeline;
    }

    public AprilTagPipeline getAprilTagPipeline() {
        return aprilTagPipeline;
    }
}