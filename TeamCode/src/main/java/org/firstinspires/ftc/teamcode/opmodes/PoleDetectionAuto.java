package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/*
This OpMode consist of moving the robot autonomously and having it choose the largest yellow object
in its field of view. It will turn to the largest one, then approach it slowly until it senses
it is close enough.

Uses the camera, contour pipeline, object detection algorithm. Uses odometry. Uses PID for movement.
 */

@Autonomous(name="Pole Detection Auto Test")
public class PoleDetectionAuto extends LinearOpMode {

    private MecanumDriveSubsystem drive;
    private IMUSubsystem imu;

    private CameraSubsystem camera;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap, CameraSubsystem.PipelineName.CONTOUR_PIPELINE);

        drive.turnOffInternalPID();
        imu.resetHeading();


        // allows telemetry to output to phone and dashboard

        FtcDashboard.getInstance().startCameraStream(camera.camera, 30);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();



        // multi threads in LinearOpmode


    }
}




