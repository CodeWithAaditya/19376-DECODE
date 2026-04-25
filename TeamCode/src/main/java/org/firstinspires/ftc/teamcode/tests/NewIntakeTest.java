package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;

@TeleOp(name = "NewIntakeTest")
public class NewIntakeTest extends LinearOpMode {

    public Intake intake = null;
    public Shooter shooter = null;
    public CRServo intakeServo = null;

    @Override
    public void runOpMode() {

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            intake.lowerSwingArm();

            if(gamepad1.dpad_right){
                shooter.setShooterVelocity(1500);
            } else if (gamepad1.dpad_left) {
                shooter.setShooterVelocity(0);
            }

            if (gamepad1.x) {
                intake.setIntakeState(Intake.IntakeState.ON);
                intakeServo.setPower(-1.0);
            } else if (gamepad1.b) {
                intake.setIntakeState(Intake.IntakeState.ON);
                intakeServo.setPower(1.0);
            } else if (gamepad1.a) {
                intake.setIntakeState(Intake.IntakeState.OFF);
                intakeServo.setPower(0.0);
            }

            if (gamepad1.dpad_up){
                intake.raiseSwingArm();
            } else if (gamepad1.dpad_down) {
                intake.lowerSwingArm();
            }

            shooter.update();
        }
    }
}