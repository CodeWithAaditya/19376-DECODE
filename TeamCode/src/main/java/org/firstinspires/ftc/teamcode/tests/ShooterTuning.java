package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.icu.util.MeasureUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive;

@TeleOp(name = "Shooter Tuner", group = "Test")
public class ShooterTuning extends LinearOpMode {

    private Shooter shooter;
    private Intake intake;
    private MecanumDrive drive;

    // Increment amounts
    private static final double HOOD_STEP = 0.01;
    private static final double FLYWHEEL_STEP = 50.0;



    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new MecanumDrive(hardwareMap);


        // Initialize hood and shooter speed
        double hoodPos = shooter.getHoodServoPos();
        double flywheelVel = shooter.getShooterVelocity();

        telemetry.addLine("Shooter Tuner Ready!");
        telemetry.addLine("DPAD UP/DOWN: Hood + / -");
        telemetry.addLine("DPAD LEFT/RIGHT: Flywheel + / -");
        telemetry.addLine("X: Reset Hood | Y: Stop Flywheel");
        telemetry.addLine("A: Print distance to goal");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.odo.update();
            if(gamepad1.right_trigger>0.5){
                intake.setIntakeState(Intake.IntakeState.ON);
            }else {
                intake.setIntakeState(Intake.IntakeState.OFF);
            }

            // Hood increment
            if (gamepad1.dpad_up) {
                hoodPos += HOOD_STEP;
            }
            if (gamepad1.dpad_down) {
                hoodPos -= HOOD_STEP;
            }

            // Flywheel increment
            if (gamepad1.dpad_right) {
                flywheelVel += FLYWHEEL_STEP;
            }
            if (gamepad1.dpad_left) {
                flywheelVel -= FLYWHEEL_STEP;
            }

            // Clamp values
            hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
            flywheelVel = Math.max(0.0, flywheelVel);

            // Quick resets
            if (gamepad1.x) hoodPos = 0.0;
            if (gamepad1.y) flywheelVel = 0.0;

            // Apply changes
            shooter.setHoodServoPos(hoodPos);
            shooter.setShooterVelocity(flywheelVel);

            shooter.update();
            Pose2D goalPose  = new Pose2D(INCH, 72, 72, DEGREES, 0); // example goal

            double distance = shooter.distanceToGoal(drive.getPose(), goalPose);

            // Telemetry
            telemetry.addData("Hood Position", "%.2f", hoodPos);
            telemetry.addData("Flywheel Velocity", "%.0f ticks/s", flywheelVel);
            telemetry.addData("Distance to Goal", "%.2f in", distance);
            telemetry.update();
        }
    }
}
