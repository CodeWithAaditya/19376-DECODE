package org.firstinspires.ftc.teamcode.tests;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Shooter;

@Configurable
@TeleOp(name = "ShooterTurretPanelsTuner")
public class ShooterTurretPIDTuning extends LinearOpMode {

    private Shooter shooter;

    // ===============================
    // CONFIGURABLE FROM PANELS
    // ===============================

    public static double shooterTargetVelocity = 0;
    public static double turretTargetAngle = 0;

    public static double shooter_kP = 0.003;
    public static double shooter_kI = 0.0;
    public static double shooter_kD = 0.00005;
    public static double shooter_kF = 0.0005;

    public static double turret_kP = 0.010;
    public static double turret_kI = 0.0005;
    public static double turret_kD = 0.0005;
    public static double turret_kF = 0.025;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void runOpMode() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Apply PID from Panels
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

            // Apply Targets
            shooter.setShooterVelocity(shooterTargetVelocity);
            shooter.setTurretAngle(-turretTargetAngle);

            shooter.update();

            // Graph Data
            telemetryM.addData("sTarget", shooterTargetVelocity);
            telemetryM.addData("sActual", shooter.getShooterVelocity());

            telemetryM.addData("tTarget", turretTargetAngle);
            telemetryM.addData("tActual", shooter.getTurretAngle());

            telemetryM.update(telemetry);
        }
    }
}
