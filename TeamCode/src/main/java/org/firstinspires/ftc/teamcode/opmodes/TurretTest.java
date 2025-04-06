package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Disabled
@TeleOp(name="Turret Test")
public class TurretTest extends OpMode {

    private TurretSubsystem turret;

    @Override
    public void init() {
        turret = new TurretSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        turret.turn(gamepad1.right_stick_x);
    }
}