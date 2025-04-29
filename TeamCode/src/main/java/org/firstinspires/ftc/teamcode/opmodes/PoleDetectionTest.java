package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp(name="Pole Detection Test")
public class PoleDetectionTest extends LinearOpMode {

    private CameraSubsystem camera;
    private TurretSubsystem turret;
    private MecanumDriveSubsystem drive;

    private IMUSubsystem imu;


    @Override
    public void runOpMode() throws InterruptedException {
        camera = new CameraSubsystem(hardwareMap, CameraSubsystem.PipelineName.CONTOUR_PIPELINE);
        turret = new TurretSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);

        imu = new IMUSubsystem(hardwareMap);

        imu.resetHeading();


        // outputting values and camera stream to FTCDashboard for debugging
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera.camera, 30);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // allows telemetry to output to phone and dashboard
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
         //   Executor executor = Executors.newFixedThreadPool(4);

        }



        waitForStart();
        while (opModeIsActive()) {
            // user controls
            if (gamepad1.right_trigger > 0) {
                turret.turn(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                turret.turn(-gamepad1.left_trigger);
            }

            //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeadingRAD());

            // reporting turret data
            telemetry.addLine("TURRET DATA");
            telemetry.addData("Turret motor power", turret.getMotorPower());
            telemetry.addData("Turret position", turret.getPosition());
            telemetry.addLine();

            // reporting the pole detection and contour data
            telemetry.addLine("CAMERA DATA");
            if (camera.getPipeline().poleDetected()) {
                // NOTE temporary list was created to prevent this OpMode thread to interfere with the Pipeline thread
                List<Double> tmp = new ArrayList<>(camera.getPipeline().getContourAreas());
                telemetry.addData("Processing time (ms)", camera.getPipeline().getProcessTime());
                telemetry.addData("Contours found", tmp.size());
                telemetry.addLine("Largest Contour Data");
                telemetry.addData("Area", camera.getPipeline().largestContourArea());
                telemetry.addData("X location", camera.getPipeline().largestContourCenter().x);
                telemetry.addData("Y location", camera.getPipeline().largestContourCenter().y);
                telemetry.addData("Camera center", CameraSubsystem.CENTER_X);
                telemetry.addLine();

                // point camera towards the detected pole
                // Reverse the direction by swapping the order of arguments
                turret.followPID((int) Math.round(camera.getPipeline().largestContourCenter().x), CameraSubsystem.CENTER_X);

            } else {
                telemetry.addLine("No contours detected");
                turret.stop();
                telemetry.update();

            }


            }


        }
    }

