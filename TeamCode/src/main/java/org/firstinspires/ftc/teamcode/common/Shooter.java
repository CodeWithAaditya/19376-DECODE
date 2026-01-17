package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Shooter {

    /* ================= Shooter Motors ================= */

    private DcMotorEx shooterMotorL;
    private DcMotor   shooterMotorR;

    /* ================= Turret Hardware ================= */

    private CRServo turretServoL;
    private CRServo turretServoR;
    private AnalogInput turretEncoder;

    private Servo hoodServo;

    /* ================= Shooter PID ================= */

    private static final double kP_SHOOT = 0.002;
    private static final double kI_SHOOT = 0.0000;
    private static final double kD_SHOOT = 0.00005;
    private static final double kF_SHOOT = 0.0005;

    private double shooterTargetVel = 0;
    private double shooterI = 0;
    private double shooterPrevErr = 0;

    /* ================= Encoder Constants ================= */

    private static final double MAX_DEGREES = 360.0;
    private static final double MAX_VOLTAGE = 3.3;

    /* ================= Turret Calibration ================= */

    private static double TURRET_OFFSET_DEG = 89.3;

    private static final double SERVO_GEAR_TEETH = 60.0;
    private static final double TURRET_GEAR_TEETH = 104.0;
    private static final double GEAR_RATIO = SERVO_GEAR_TEETH / TURRET_GEAR_TEETH;

    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG =  90.0;

    /* ================= Turret PID ================= */

    private static final double kP_TURRET = 0.02;
    private static final double kI_TURRET = 0.0;
    private static final double kD_TURRET = 0.0005;

    private static final double MAX_TURRET_POWER = 0.8;
    private static final double TURRET_DEADZONE = 0.5;

    private double turretTargetAngle = 0;
    private double turretI = 0;
    private double turretPrevErr = 0;
    private boolean turretPIDEnabled = true;

    /* ================= Encoder Unwrap State ================= */

    private double lastRawAngle = 0;
    private double unwrappedAngle = 0;

    /* ================= Misc ================= */

    private double hoodTargetPos = 0;
    private final ElapsedTime dtTimer = new ElapsedTime();

    /* ================= Init ================= */

    public Shooter(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {

        shooterMotorL = hMap.get(DcMotorEx.class, "shooterMotorL");
        shooterMotorR = hMap.get(DcMotor.class,   "shooterMotorR");

        shooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretServoL = hMap.get(CRServo.class, "turretServoL");
        turretServoR = hMap.get(CRServo.class, "turretServoR");
        turretEncoder = hMap.get(AnalogInput.class, "turretEncoder");

        hoodServo = hMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(hoodTargetPos);

        dtTimer.reset();
    }

    /* ================= Public API ================= */

    public void update() {
        double dt = dtTimer.seconds();
        dtTimer.reset();

        updateShooter(dt);
        updateTurret(dt);
    }

    public void setShooterVelocity(double ticksPerSecond) {
        shooterTargetVel = ticksPerSecond;
    }

    public void stopShooter() {
        shooterTargetVel = 0;
        shooterI = 0;
        shooterPrevErr = 0;
    }

    public double getShooterVelocity() {
        return shooterMotorL.getVelocity();
    }

    public void setTurretAngle(double degrees) {
        turretTargetAngle = Range.clip(degrees, TURRET_MIN_DEG, TURRET_MAX_DEG);
        turretPIDEnabled = true;
    }

    public void setTurretManual(double power) {
        turretPIDEnabled = false;
        turretServoL.setPower(power);
        turretServoR.setPower(-power);
    }

    public void stopTurret() {
        turretPIDEnabled = true;
        turretTargetAngle = getCorrectedTurretAngle();
        turretI = 0;
        turretPrevErr = 0;
    }

    public double getTurretAngle() {
        return getCorrectedTurretAngle();
    }

    public double getHoodServoPos() {
        return hoodServo.getPosition();
    }

    public void setHoodServoPos(double targetPos) {
        hoodTargetPos = targetPos;
        hoodServo.setPosition(hoodTargetPos);
    }

    /* ================= Shooter Control ================= */

    private void updateShooter(double dt) {

        double currentVel = shooterMotorL.getVelocity();
        double error = shooterTargetVel - currentVel;

        shooterI += error * dt;
        double derivative = (error - shooterPrevErr) / dt;
        shooterPrevErr = error;

        double output =
                kP_SHOOT * error +
                        kI_SHOOT * shooterI +
                        kD_SHOOT * derivative +
                        kF_SHOOT * shooterTargetVel;

        output = Range.clip(output, -1.0, 1.0);

        shooterMotorL.setPower(-output);
        shooterMotorR.setPower(-output);
    }

    /* ================= Turret Control ================= */

    private void updateTurret(double dt) {

        double currentAngle = getCorrectedTurretAngle();
        double power = 0;

        if (turretPIDEnabled) {

            double error = angleError(turretTargetAngle, currentAngle);

            if (Math.abs(error) < TURRET_DEADZONE) {
                turretI = 0;
            } else {
                turretI += error * dt;
                double derivative = (error - turretPrevErr) / dt;
                turretPrevErr = error;

                power =
                        kP_TURRET * error +
                                kI_TURRET * turretI +
                                kD_TURRET * derivative;

                power = Range.clip(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);
            }
        }

        // Safety hard stops
        if ((currentAngle <= TURRET_MIN_DEG && power < 0) ||
                (currentAngle >= TURRET_MAX_DEG && power > 0)) {
            power = 0;
        }

        turretServoL.setPower(power);
        turretServoR.setPower(-power);
    }

    public double distanceToGoal(Pose2D robotPose, Pose2D goalPose) {
        double dx = goalPose.getX(INCH) - robotPose.getX(INCH); // forward
        double dy = goalPose.getY(INCH) - robotPose.getY(INCH); // left
        return Math.hypot(dx, dy);
    }

    public double[] getShooterSettingsFromDistance(double distance) {

        // Shooter flywheel speed (y1)
        double flywheel =
                0.000458708 * Math.pow(distance, 3)
                        - 0.0894792 * Math.pow(distance, 2)
                        + 10.97431 * distance
                        + 974.8646;

        // Hood position (z1)
        double hood =
                6.12363e-7 * Math.pow(distance, 3)
                        - 0.000220446 * Math.pow(distance, 2)
                        + 0.0257331 * distance
                        - 0.682439;

        // Clip outputs
        flywheel = Math.max(0, Math.min(2200, flywheel));
        hood = Math.max(0, Math.min(0.35, hood));

        return new double[] { flywheel, hood };
    }

    public double autoAimTurretAngle(Pose2D robotPose, Pose2D goalPose) {
        double dx = goalPose.getX(INCH) - robotPose.getX(INCH);
        double dy = goalPose.getY(INCH) - robotPose.getY(INCH);

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);


        return Range.clip(fieldAngleDeg - robotHeadingDeg, -90.0, 90.0);
    }

    /* ================= Encoder Math ================= */

    private double getRawServoAngle() {
        return (turretEncoder.getVoltage() / MAX_VOLTAGE) * MAX_DEGREES;
    }

    private double getUnwrappedServoAngle() {
        double raw = getRawServoAngle();
        double delta = raw - lastRawAngle;

        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        unwrappedAngle += delta;
        lastRawAngle = raw;

        return unwrappedAngle;
    }

    private double getCorrectedTurretAngle() {
        return getUnwrappedServoAngle() * GEAR_RATIO - TURRET_OFFSET_DEG;
    }

    /* ================= Math Helpers ================= */

    private double angleError(double target, double current) {
        return target - current;
    }
}
