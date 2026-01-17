package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Turret Test Revcoder", group = "Test")
public class TurretTestRevcoder extends LinearOpMode {

    /* ================= Hardware ================= */

    private CRServo turretServoL;
    private CRServo turretServoR;
    private DcMotorEx turretEncoder;

    /* ================= Encoder Constants ================= */

    private static final double TICKS_PER_REV = 8192.0;

    private static final double ENCODER_GEAR_TEETH = 25.0;
    private static final double TURRET_GEAR_TEETH  = 104.0;

    private static final double GEAR_RATIO =
            ENCODER_GEAR_TEETH / TURRET_GEAR_TEETH;

    /* ================= Turret Limits ================= */

    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG =  90.0;

    /* ================= PID Constants ================= */

    private static final double kP = 0.025;
    private static final double kI = 0.0;
    private static final double kD = 0.0005;

    private static final double MAX_POWER = 1.0;
    private static final double DEADZONE_DEG = 0.5;

    /* ================= State ================= */

    private double turretTargetDeg = 0.0;
    private double turretOffsetDeg = 0.0;

    private double turretI = 0.0;
    private double turretPrevErr = 0.0;

    private boolean pidEnabled = true;

    private final ElapsedTime dtTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* ================= Init ================= */

        turretServoL = hardwareMap.get(CRServo.class, "turretServoL");
        turretServoR = hardwareMap.get(CRServo.class, "turretServoR");

        turretEncoder = hardwareMap.get(DcMotorEx.class, "driveFL");

        turretEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Turret FULL Test Ready");
        telemetry.addLine("Y = Zero | A/B/X = 0/+45/-45");
        telemetry.update();

        waitForStart();
        dtTimer.reset();

        /* ================= Loop ================= */

        while (opModeIsActive()) {

            double dt = dtTimer.seconds();
            dtTimer.reset();

            /* ================= Encoder → Angle ================= */

            int ticks = turretEncoder.getCurrentPosition();
            double encoderDeg = (ticks / TICKS_PER_REV) * 360.0;
            double turretAngleDeg = encoderDeg * GEAR_RATIO - turretOffsetDeg;

            /* ================= Zeroing ================= */

            if (gamepad1.y) {

                turretOffsetDeg = 0.0;
                turretTargetDeg = 0.0;

                turretI = 0;
                turretPrevErr = 0;
            }

            /* ================= Manual Control ================= */

            double manualPower = gamepad1.left_stick_x;

            if (Math.abs(manualPower) > 0.1) {
                pidEnabled = false;
                turretServoL.setPower(manualPower);
                turretServoR.setPower(-manualPower);
            } else {
                pidEnabled = true;
            }

            /* ================= PID Target Selection ================= */

            if (gamepad1.a) turretTargetDeg = 0.0;
            if (gamepad1.b) turretTargetDeg = 45.0;
            if (gamepad1.x) turretTargetDeg = -45.0;

            if (gamepad1.right_bumper) {
                turretTargetDeg = turretAngleDeg;
                turretI = 0;
                turretPrevErr = 0;
            }

            turretTargetDeg = Range.clip(
                    turretTargetDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);

            /* ================= PID Control ================= */

            double power = 0.0;

            if (pidEnabled) {

                double error = turretTargetDeg - turretAngleDeg;

                if (Math.abs(error) < DEADZONE_DEG) {
                    turretI = 0;
                } else {
                    turretI += error * dt;
                    double derivative = (error - turretPrevErr) / dt;
                    turretPrevErr = error;

                    power = kP * error + kI * turretI + kD * derivative;
                    power = Range.clip(power, -MAX_POWER, MAX_POWER);
                }
            }

            /* ================= Safety Stops ================= */

            if ((turretAngleDeg <= TURRET_MIN_DEG && power < 0) ||
                    (turretAngleDeg >= TURRET_MAX_DEG && power > 0)) {
                power = 0;
            }

            turretServoL.setPower(-power);
            turretServoR.setPower(power);

            /* ================= Telemetry ================= */

            telemetry.addData("Encoder Ticks", ticks);
            telemetry.addData("Turret Angle (deg)", "%.2f", turretAngleDeg);
            telemetry.addData("Target Angle (deg)", "%.2f", turretTargetDeg);
            telemetry.addData("PID Enabled", pidEnabled);
            telemetry.addData("Power", "%.2f", power);

            telemetry.update();
        }
    }
}
