package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.Constants;

@Configurable
@TeleOp(name = "ShooterTunerV2", group = "Test")
public class ShooterTunerV2 extends LinearOpMode {

    private Shooter shooter;
    private Intake intake;
    private Follower follower;

    // ===============================
    // PANELS CONFIGURABLE VARIABLES
    // ===============================

    // Targets
    public static double shooterTargetVelocity = 0.0;
    public static double turretTargetAngle = 0.0;

    // Shooter PID
    public static double shooter_kP = 0.02;
    public static double shooter_kI = 0.0;
    public static double shooter_kD = 0.00008;
    public static double shooter_kF = 0.0005;

    // Turret PID
    public static double turret_kP = 0.010;
    public static double turret_kI = 0.0006;
    public static double turret_kD = 0.0005;
    public static double turret_kF = 0.020;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    // Goal position (edit if needed)
    private Pose goalPose = new Pose(144, 144, 0);

    @Override
    public void runOpMode() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112, 135, Math.toRadians(-90)));
        follower.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            // ===============================
            // GAMEPAD INTAKE CONTROL
            // ===============================

            if (gamepad1.right_trigger > 0.5) {
                intake.setIntakeState(Intake.IntakeState.ON);
            } else {
                intake.setIntakeState(Intake.IntakeState.OFF);
            }

            // ===============================
            // APPLY PID FROM PANELS
            // ===============================

            shooter.configureShooterPID(
                    shooter_kP,
                    shooter_kI,
                    shooter_kD,
                    shooter_kF
            );

            shooter.configureTurretPID(
                    turret_kP,
                    turret_kI,
                    turret_kD,
                    turret_kF
            );

            // ===============================
            // APPLY TARGETS FROM PANELS
            // ===============================

            shooter.setShooterVelocity(shooterTargetVelocity);

//            shooter.setTurretAngle(shooter.autoAimTurretAngle(follower.getPose(), goalPose));
            shooter.setTurretAngle(-turretTargetAngle);

            shooter.update();

            // ===============================
            // CALCULATIONS
            // ===============================

            double distance =
                    shooter.distanceToGoal(
                            follower.getPose(),
                            goalPose
                    );

            double actualVelocity = shooter.getShooterVelocity();
            double actualTurret = shooter.getTurretAngle();

            // ===============================
            // PANELS TELEMETRY
            // ===============================

            telemetryM.addData("DistanceToGoal_in", distance);

            telemetryM.addData("ShooterTarget", shooterTargetVelocity);
            telemetryM.addData("ShooterActual", actualVelocity);

            telemetryM.addData("TurretTarget_deg", turretTargetAngle);
            telemetryM.addData("TurretActual_deg", actualTurret);

            telemetryM.update(telemetry);
        }
    }
}
