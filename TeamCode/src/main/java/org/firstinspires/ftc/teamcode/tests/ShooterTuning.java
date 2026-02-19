package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.icu.util.MeasureUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.Constants;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.PoseStorage;

@Disabled
@TeleOp(name = "Shooter Tuner", group = "Test")
public class ShooterTuning extends LinearOpMode {

    private Shooter shooter;
    private Intake intake;
    private Follower follower;

    // Increment amounts
    private static final double HOOD_STEP = 0.01;
    private static final double FLYWHEEL_STEP = 25;



    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112, 135, Math.toRadians(-90)));
        follower.update();

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
            follower.update();
            if(gamepad1.right_trigger>0.5){
                intake.setIntakeState(Intake.IntakeState.ON);
            }else {
                intake.setIntakeState(Intake.IntakeState.OFF);
            }

            // Hood increment
            if (gamepad1.dpadUpWasPressed()) {
                hoodPos += HOOD_STEP;
            }
            if (gamepad1.dpadDownWasPressed()) {
                hoodPos -= HOOD_STEP;
            }

            // Flywheel increment
            if (gamepad1.dpadRightWasPressed()) {
                flywheelVel += FLYWHEEL_STEP;
            }
            if (gamepad1.dpadLeftWasPressed()) {
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
            Pose goalPose  = new Pose(144, 144, 0); // example goal

            double distance = shooter.distanceToGoal(follower.getPose(), goalPose);

            // Telemetry
            telemetry.addData("Hood Position", "%.2f", hoodPos);
            telemetry.addData("Flywheel Velocity", "%.0f ticks/s", flywheelVel);
            telemetry.addData("Distance to Goal", "%.2f in", distance);
            telemetry.update();
        }
    }
}
