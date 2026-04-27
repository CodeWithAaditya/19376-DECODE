package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Shooter {

    private DcMotorEx shooterMotorL;
    private DcMotorEx shooterMotorR;

    private CRServo turretServoL;
    private CRServo turretServoR;

    private static double kP_SHOOT = 0.02;
    private static double kI_SHOOT = 0.0;
    private static double kD_SHOOT = 0.00008;
    private static double kF_SHOOT = 0.0005;

    private double shooterTargetVel = 0;
    private double shooterI = 0;
    private double shooterPrevErr = 0;

    private static final double TICKS_PER_REV = 8192.0;
    private static final double ENCODER_GEAR_TEETH = 30.0;
    private static final double TURRET_GEAR_TEETH  = 92.0;
    private static final double GEAR_RATIO = ENCODER_GEAR_TEETH / TURRET_GEAR_TEETH;

    private static double TURRET_OFFSET_DEG = 0.0;

    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG =  90.0;

    private static double kP_TURRET = 0.01;
    private static double kI_TURRET = 0.0006;
    private static double kD_TURRET = 0.0005;
    private static double kF_TURRET = 0.020;

//    private static double kP_TURRET = 0.008;
//    private static double kI_TURRET = 0.0007;
//    private static double kD_TURRET = 0.0008;
//    private static double kF_TURRET = 0.032;


    private static final double MAX_TURRET_POWER = 1.0;

    private double turretTargetAngle = 0;
    private double turretI = 0;
    private double turretPrevErr = 0;
    private boolean turretPIDEnabled = true;

    private final ElapsedTime dtTimer = new ElapsedTime();

    public Shooter(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {

        shooterMotorL = hMap.get(DcMotorEx.class, "shooterMotorL");
        shooterMotorR = hMap.get(DcMotorEx.class, "shooterMotorR");

        shooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretServoL = hMap.get(CRServo.class, "turretServoL");
        turretServoR = hMap.get(CRServo.class, "turretServoR");

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
        return -shooterMotorL.getVelocity();
    }

    private void updateShooter(double dt) {

        double currentVel = getShooterVelocity();
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
        turretTargetAngle = Range.clip(-degrees, TURRET_MIN_DEG, TURRET_MAX_DEG);
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

        double error = turretTargetAngle - currentAngle;

        turretI += error * dt;
        double derivative = (error - turretPrevErr) / dt;
        turretPrevErr = error;

        double rawPID =
                kP_TURRET * error +
                        kI_TURRET * turretI +
                        kD_TURRET * derivative;

        double ff = kF_TURRET * Math.signum(error);

        power = rawPID + ff;
        power = Range.clip(power, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        if ((currentAngle <= TURRET_MIN_DEG && power < 0) ||
                (currentAngle >= TURRET_MAX_DEG && power > 0)) {
            power = 0;
        }

        turretServoL.setPower(-power);
        turretServoR.setPower(power);
    }

    private double ticksToDegrees(double ticks) {
        return (ticks / TICKS_PER_REV) * 360.0;
    }

    private double getCorrectedTurretAngle() {
        double encoderTicks = shooterMotorR.getCurrentPosition();
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

    public double[] SOTMTurretAngle(Pose robotPose, Pose goalPose, Vector robotVelocity) {
        double shotTime = shotTimeFromDistance(distanceToGoal(robotPose, goalPose));
        Pose correctedGoalPos = new Pose(goalPose.getX()-robotVelocity.getXComponent()*shotTime*1.2, goalPose.getY()-robotVelocity.getYComponent()*shotTime*1.2, goalPose.getHeading());
        double[] correctedShooterSettings =
                getShooterSettingsFromDistance(distanceToGoal(robotPose, correctedGoalPos));

        return new double[] {autoAimTurretAngle(robotPose, correctedGoalPos), correctedShooterSettings[0]};
    }

    public double shotTimeFromDistance(double distance) {

        double shotTime =
                8.7886e-7 * Math.pow(distance, 3)
                        - 0.000264392 * Math.pow(distance, 2)
                        + 0.0309481 * distance
                        - 0.619251;

        shotTime = Range.clip(shotTime, 0.38, 1.1);

        return shotTime;
    }

//    public double autoAimTurretAngle(Pose robotPose, Pose goalPose) {
//
//        double dx = goalPose.getX() - robotPose.getX();
//        double dy = goalPose.getY() - robotPose.getY();
//
//        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
//        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
//
//        return Range.clip(fieldAngleDeg - robotHeadingDeg,
//                TURRET_MIN_DEG, TURRET_MAX_DEG);
//    }

    public double autoAimTurretAngle(Pose robotPose, Pose goalPose) {

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        // Normalize both angles to [0,360)
        fieldAngleDeg = wrap360(fieldAngleDeg);
        robotHeadingDeg = wrap360(robotHeadingDeg);

        double relativeAngle = fieldAngleDeg - robotHeadingDeg;

        // Keep relative angle continuous (no 360 jumps)
        if (relativeAngle > 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        // HARD mechanical limit
        return Range.clip(relativeAngle, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    private double wrap360(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    public double distanceToGoal(Pose robotPose, Pose goalPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    public double[] getShooterSettingsFromDistance(double distance) {

        double flywheel =
                0.000647334 * Math.pow(distance, 3)
                        - 0.154057 * Math.pow(distance, 2)
                        + 18.43132 * distance
                        + 170.94439;

        flywheel = Range.clip(flywheel, 0, 1650);

        return new double[] { flywheel };
    }

    public void configureShooterPID(double p, double i, double d, double f) {
        kP_SHOOT = p;
        kI_SHOOT = i;
        kD_SHOOT = d;
        kF_SHOOT = f;
    }

    public void configureTurretPID(double p, double i, double d, double f) {
        kP_TURRET = p;
        kI_TURRET = i;
        kD_TURRET = d;
        kF_TURRET = f;
    }
}