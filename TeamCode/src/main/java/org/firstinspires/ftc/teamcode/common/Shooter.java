package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Shooter {

    private DcMotorEx shooterMotorL;
    private DcMotorEx shooterMotorR;

    private CRServo turretServoL;
    private CRServo turretServoR;
    private DcMotorEx turretEncoder;

    private Servo hoodServo;

    private static final double kP_SHOOT = 0.002;
    private static final double kI_SHOOT = 0.0;
    private static final double kD_SHOOT = 0.00005;
    private static final double kF_SHOOT = 0.0005;

    private double shooterTargetVel = 0;
    private double shooterI = 0;
    private double shooterPrevErr = 0;

    private static final double TICKS_PER_REV = 8192.0;
    private static final double ENCODER_GEAR_TEETH = 25.0;
    private static final double TURRET_GEAR_TEETH  = 104.0;
    private static final double GEAR_RATIO = ENCODER_GEAR_TEETH / TURRET_GEAR_TEETH;

    private static double TURRET_OFFSET_DEG = 0.0;

    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG =  90.0;

    private static final double kP_TURRET = 0.025;
    private static final double kI_TURRET = 0.0;
    private static final double kD_TURRET = 0.0005;

    private static final double MAX_TURRET_POWER = 1.0;
    private static final double TURRET_DEADZONE = 0.5;

    private double turretTargetAngle = 0;
    private double turretI = 0;
    private double turretPrevErr = 0;
    private boolean turretPIDEnabled = true;

    private double hoodTargetPos = 0.0;

    private final ElapsedTime dtTimer = new ElapsedTime();

    public Shooter(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {

        shooterMotorL = hMap.get(DcMotorEx.class, "shooterMotorL");
        shooterMotorR = hMap.get(DcMotorEx.class, "shooterMotorR");

        shooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretServoL = hMap.get(CRServo.class, "turretServoL");
        turretServoR = hMap.get(CRServo.class, "turretServoR");

        turretEncoder = hMap.get(DcMotorEx.class, "driveFL");

        hoodServo = hMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(hoodTargetPos);

        dtTimer.reset();
    }

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

    public void setTurretAngle(double degrees) {
        turretTargetAngle = Range.clip(-degrees, TURRET_MIN_DEG, TURRET_MAX_DEG); //negative on degrees for some nonsense
        turretPIDEnabled = true;
    }

    public void setTurretManual(double power) {
        turretPIDEnabled = false;
        turretServoL.setPower(power);
        turretServoR.setPower(-power);
    }

    public void stopTurret() {
        turretPIDEnabled = true;
        turretTargetAngle = getTurretAngle();
        turretI = 0;
        turretPrevErr = 0;
    }

    public double getTurretAngle() {
        return getCorrectedTurretAngle();
    }

    private void updateTurret(double dt) {

        double currentAngle = getCorrectedTurretAngle();
        double power = 0;

        if (turretPIDEnabled) {

            double error = turretTargetAngle - currentAngle;

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

        if ((currentAngle <= TURRET_MIN_DEG && power < 0) ||
                (currentAngle >= TURRET_MAX_DEG && power > 0)) {
            power = 0;
        }

        turretServoL.setPower(-power);
        turretServoR.setPower(power);
    }

    public double getHoodServoPos() {
        return hoodServo.getPosition();
    }

    public void setHoodServoPos(double targetPos) {
        hoodTargetPos = Range.clip(targetPos, 0.0, 1.0);
        hoodServo.setPosition(hoodTargetPos);
    }

    private double ticksToDegrees(double ticks) {
        return (ticks / TICKS_PER_REV) * 360.0;
    }

    private double getCorrectedTurretAngle() {
        double encoderTicks = turretEncoder.getCurrentPosition();
        double encoderDeg = ticksToDegrees(encoderTicks);
        return encoderDeg * GEAR_RATIO - TURRET_OFFSET_DEG;
    }

    public void trimTurretOffset(double deltaDeg) {
        TURRET_OFFSET_DEG += deltaDeg;
    }

    public void setTurretOffset(double offsetDeg) {
        TURRET_OFFSET_DEG = offsetDeg;
    }

    public double getTurretOffset() {
        return TURRET_OFFSET_DEG;
    }

    public double autoAimTurretAngle(Pose2D robotPose, Pose2D goalPose) {

        double dx = goalPose.getX(INCH) - robotPose.getX(INCH)-3;
        double dy = goalPose.getY(INCH) - robotPose.getY(INCH);

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

        return Range.clip(fieldAngleDeg - robotHeadingDeg,
                TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    public double distanceToGoal(Pose2D robotPose, Pose2D goalPose) {
        double dx = goalPose.getX(INCH) - robotPose.getX(INCH)-3;
        double dy = goalPose.getY(INCH) - robotPose.getY(INCH);
        return Math.hypot(dx, dy);
    }

    public double[] getShooterSettingsFromDistance(double distance) {

        double flywheel =
                0.000458708 * Math.pow(distance, 3)
                        - 0.0894792 * Math.pow(distance, 2)
                        + 10.97431 * distance
                        + 974.8646;

        double hood =
                6.12363e-7 * Math.pow(distance, 3)
                        - 0.000220446 * Math.pow(distance, 2)
                        + 0.0257331 * distance
                        - 0.682439;

        flywheel = Range.clip(flywheel, 0, 2200);
        hood = Range.clip(hood, 0, 0.35);

        return new double[] { flywheel, hood };
    }
}