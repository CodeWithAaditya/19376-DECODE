package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Shooter {

    private DcMotorEx shooterMotorL;
    private DcMotor   shooterMotorR;

    private CRServo turretServoL;
    private CRServo turretServoR;
    private AnalogInput turretEncoder;

    private Servo hoodServo;

    private static final double kP_SHOOT = 0.002;
    private static final double kI_SHOOT = 0.0000;
    private static final double kD_SHOOT = 0.00005;
    private static final double kF_SHOOT = 0.0005;

    private double shooterTargetVel = 0;
    private double shooterI = 0;
    private double shooterPrevErr = 0;


    private static final double MAX_DEGREES = 360.0;
    private static final double MAX_VOLTAGE = 3.3;

    private static double TURRET_OFFSET_DEG = 37.0;

    private static final double kP_TURRET = 0.015;
    private static final double kI_TURRET = 0.0;
    private static final double kD_TURRET = 0.0005;

    private static final double MAX_TURRET_POWER = 0.6;
    private static final double TURRET_DEADZONE = 0.5;

    private double turretTargetAngle = 0;
    private double turretI = 0;
    private double turretPrevErr = 0;
    private boolean turretPIDEnabled = true;

    private double hoodTargetPos = 0;

    private final ElapsedTime dtTimer = new ElapsedTime();

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

    public double getHoodServoPos(){
        return hoodServo.getPosition();
    }

    public void setHoodServoPos(double targetPos){
        hoodTargetPos = targetPos;
        hoodServo.setPosition(hoodTargetPos);
    }

    public void update() {
        double dt = dtTimer.seconds();
        dtTimer.reset();

        updateShooter(dt);
        updateTurret(dt);
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

    private void updateTurret(double dt) {

        double currentAngle = getCorrectedTurretAngle();
        double power;

        if (turretPIDEnabled) {
            double error = angleError(turretTargetAngle, currentAngle);

            if (Math.abs(error) < TURRET_DEADZONE) {
                turretI = 0;
                power = 0;
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
        } else {
            power = 0;
        }

        turretServoL.setPower(power);
        turretServoR.setPower(-power);
    }

    public void setTurretAngle(double degrees) {
        turretTargetAngle = wrapAngle(degrees);
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

    private double getRawTurretAngle() {
        return (turretEncoder.getVoltage() / MAX_VOLTAGE) * MAX_DEGREES;
    }

    private double getCorrectedTurretAngle() {
        double angle = getRawTurretAngle() - TURRET_OFFSET_DEG;
        return wrapAngle(angle);
    }

    private double wrapAngle(double angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }

    private double angleError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
