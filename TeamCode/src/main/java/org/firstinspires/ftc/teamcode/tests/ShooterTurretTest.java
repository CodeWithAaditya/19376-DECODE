package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Shooter;

@TeleOp(name = "Shooter Turret Test")
@Disabled
public class ShooterTurretTest extends LinearOpMode {
    Shooter shooter;

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) shooter.stopShooter();
            if (gamepad1.x) shooter.setShooterVelocity(1400);
            if (gamepad1.y) shooter.setShooterVelocity(2000);
            if (gamepad1.b) shooter.setShooterVelocity(2800);

            if (gamepad1.dpad_left)  shooter.setTurretAngle(0);
            if (gamepad1.dpad_right) shooter.setTurretAngle(45);

            if(gamepad1.dpad_up){
                shooter.setHoodServoPos(shooter.getHoodServoPos()+0.01);
                sleep(100);
            }
            if(gamepad1.dpad_down){
                shooter.setHoodServoPos(shooter.getHoodServoPos()-0.01);
                sleep(100);
            }

            shooter.update();
        }
    }

}
