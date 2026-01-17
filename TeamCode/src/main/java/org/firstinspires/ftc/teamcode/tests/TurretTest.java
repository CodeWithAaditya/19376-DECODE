package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Test", group = "Test")
@Disabled
public class TurretTest extends LinearOpMode {

    /* ================= Hardware ================= */

    private CRServo turretServoL;
    private CRServo turretServoR;
    private AnalogInput turretEncoder;

    /* ================= Encoder Constants ================= */

    private static final double MAX_DEGREES = 360.0;
    private static final double MAX_VOLTAGE = 3.3;

    /* ================= OFFSET (SET AFTER CALIBRATION) ================= */

    // 👇 CHANGE THIS ONCE CALIBRATED
    private static double TURRET_OFFSET_DEG = 21.3461538462;

    private double lastRawAngle = 0;
    private double unwrappedAngle = 0;

    /* ================= PID Constants ================= */

    private static double kP = 0.03;
    private static double kI = 0.0;
    private static double kD = 0.0005;

    private static final double MAX_POWER = 0.6;
    private static final double DEADZONE_DEG = 0.5;

    /* ================= PID State ================= */

    private double targetAngle = 0;
    private double integral = 0;
    private double prevError = 0;

    private boolean pidEnabled = true;
    private final ElapsedTime timer = new ElapsedTime();

    /* ================= OpMode ================= */

    @Override
    public void runOpMode() {

        turretServoL = hardwareMap.get(CRServo.class, "turretServoL");
        turretServoR = hardwareMap.get(CRServo.class, "turretServoR");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        telemetry.addLine("Turret PID Test (With Offset)");
        telemetry.addLine("A = PID ON | B = PID OFF");
        telemetry.addLine("DPAD L/R = Target Angle");
        telemetry.addLine("DPAD U/D = Adjust Offset");
        telemetry.addLine("X = 0° | Y = 180°");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            double dt = timer.seconds();
            timer.reset();

            /* ================= Gamepad Input ================= */

            if (gamepad1.dpad_right) targetAngle += 5;
            if (gamepad1.dpad_left)  targetAngle -= 5;

            if (gamepad1.x) targetAngle = 0;
            if (gamepad1.y) targetAngle = 45;



            if (gamepad1.a) pidEnabled = true;
            if (gamepad1.b) pidEnabled = false;

            // 🔧 Offset calibration
            if (gamepad1.dpad_up)   TURRET_OFFSET_DEG += 0.5;
            if (gamepad1.dpad_down) TURRET_OFFSET_DEG -= 0.5;

            /* ================= Read Encoder ================= */

            double rawAngle = getRawTurretAngle();
            double currentAngle = getCorrectedTurretAngle();

            double power;

            /* ================= PID Control ================= */

            if (pidEnabled) {
                double error = angleError(targetAngle, currentAngle);

                if (Math.abs(error) < DEADZONE_DEG) {
                    integral = 0;
                    power = 0;
                } else {
                    integral += error * dt;
                    double derivative = (error - prevError) / dt;
                    prevError = error;

                    power = kP * error
                            + kI * integral
                            + kD * derivative;

                    power = Range.clip(power, -MAX_POWER, MAX_POWER);
                }
            } else {
                // Manual override
                power = -gamepad1.left_stick_x;
                integral = 0;
                prevError = 0;
            }

            turretServoL.setPower(power);
            turretServoR.setPower(-power);

            /* ================= Telemetry ================= */

            telemetry.addData("PID Enabled", pidEnabled);
            telemetry.addData("Target Angle", "%.1f°", targetAngle);
            telemetry.addData("Raw Angle", "%.1f°", rawAngle);
            telemetry.addData("Corrected Angle", "%.1f°", currentAngle);
            telemetry.addData("Offset", "%.1f°", TURRET_OFFSET_DEG);
            telemetry.addData("Error", "%.1f°", angleError(targetAngle, currentAngle));
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Voltage", "%.3f V", turretEncoder.getVoltage());
            telemetry.addData("kP / kI / kD", "%.3f / %.3f / %.3f", kP, kI, kD);
            telemetry.update();
        }

        turretServoL.setPower(0);
        turretServoR.setPower(0);
    }

    /* ================= Helper Methods ================= */

    private double getRawTurretAngle() {
        double raw = (turretEncoder.getVoltage() / MAX_VOLTAGE) * MAX_DEGREES;
        double delta = raw - lastRawAngle;

        // Detect wraparound
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        unwrappedAngle += delta;
        lastRawAngle = raw;

        return unwrappedAngle;
    }

    private double getCorrectedTurretAngle() {
        double corrected = getRawTurretAngle()*(60.0/104.0) - TURRET_OFFSET_DEG;

//        while (corrected < 0) corrected += 360;
//        while (corrected >= 360) corrected -= 360;

        return corrected;
    }

    // Shortest-path angle error (prevents long spins)
//    private double angleError(double target, double current) {
//        double error = target - current;
//        while (error > 180) error -= 360;
//        while (error < -180) error += 360;
//        return error;
//    }

    private double angleError(double target, double current) {
        return target - current;
    }
}
