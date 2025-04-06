package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDController;

public class TurretSubsystem {

    private final DcMotor turret;
    private final PIDController followPID;
    private final double SCALE = 0.3;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turret");

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // disables the built in PID controller in the motor
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // this PID controller is tuned for following the yellow poles
        followPID = new PIDController(0.002, 0, 0.00001);
    }

    // general turn method that turns by a power
    public void turn(double pow) {
        turret.setPower(pow * SCALE);
    }

    // turn method that turns to the desired position, this is a PID movement
    public void followPID(double current, double target) {
        turret.setPower(followPID.PIDOutput(current, target));
    }

    public void stop() {
        turret.setPower(0);
    }

    // methods for OpMode telemetry
    public double getMotorPower() {
        return turret.getPower();
    }
    public int getPosition() {
        return turret.getCurrentPosition();
    }
}