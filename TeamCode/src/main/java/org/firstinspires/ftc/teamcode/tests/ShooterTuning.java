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

    private static final double FLYWHEEL_STEP = 25;



    @Override
    public void runOpMode() {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112, 135, Math.toRadians(-90)));
        follower.update();

        double flywheelVel = shooter.getShooterVelocity();

        telemetry.addLine("Shooter Tuner Ready!");
        telemetry.addLine("DPAD LEFT/RIGHT: Flywheel + / -");
        telemetry.addLine("Y: Stop Flywheel");
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

            // Flywheel increment
            if (gamepad1.dpadRightWasPressed()) {
                flywheelVel += FLYWHEEL_STEP;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                flywheelVel -= FLYWHEEL_STEP;
            }

            // Clamp values
            flywheelVel = Math.max(0.0, flywheelVel);

            // Quick resets
            if (gamepad1.y) flywheelVel = 0.0;

            // Apply changes
            shooter.setShooterVelocity(flywheelVel);

            shooter.update();
            Pose goalPose  = new Pose(144, 144, 0); // example goal

            double distance = shooter.distanceToGoal(follower.getPose(), goalPose);

            // Telemetry
            telemetry.addData("Flywheel Velocity", "%.0f ticks/s", flywheelVel);
            telemetry.addData("Distance to Goal", "%.2f in", distance);
            telemetry.update();
        }
    }
}
