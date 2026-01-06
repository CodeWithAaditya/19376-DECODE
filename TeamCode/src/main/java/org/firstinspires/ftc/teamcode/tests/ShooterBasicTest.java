package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Shooter Basic Test", group = "Shooter")
public class ShooterBasicTest extends LinearOpMode {

    DcMotorEx shooterMotorL;   // encoder plugged in
    DcMotor   shooterMotorR;   // no encoder

    // === PIDF COEFFICIENTS (TUNE THESE) ===
    static double kP = 0.002;
    static double kI = 0.0000;
    static double kD = 0.00005;
    static double kF = 0.0005;

    // Target velocities (ticks/sec)
    static final double LOW  = 1400;
    static final double MID  = 2000;
    static final double HIGH = 2800;

    double targetVelocity = 0;

    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        shooterMotorL = hardwareMap.get(DcMotorEx.class, "shooterMotorL");
        shooterMotorR = hardwareMap.get(DcMotor.class, "shooterMotorR");

        shooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Custom Shooter PID Ready");
        telemetry.addLine("A=Stop | X=Low | Y=Mid | B=High");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            // Target selection
            if (gamepad1.a) targetVelocity = 0;
            if (gamepad1.x) targetVelocity = LOW;
            if (gamepad1.y) targetVelocity = MID;
            if (gamepad1.b) targetVelocity = HIGH;

            // Measured velocity (ticks/sec)
            double currentVelocity = shooterMotorL.getVelocity();

            // PID calculations
            double error = targetVelocity - currentVelocity;

            double dt = timer.seconds();
            timer.reset();

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double output =
                    (kP * error) +
                            (kI * integralSum) +
                            (kD * derivative) +
                            (kF * targetVelocity);

            // Clamp power
            output = Math.max(-1.0, Math.min(1.0, output));

            shooterMotorL.setPower(-output);
            shooterMotorR.setPower(-output);

            telemetry.addData("Target", "%.0f ticks/s", targetVelocity);
            telemetry.addData("Velocity", "%.0f ticks/s", currentVelocity);
            telemetry.addData("Error", "%.0f", error);
            telemetry.addData("Power", "%.2f", output);
            telemetry.update();
        }
    }
}
