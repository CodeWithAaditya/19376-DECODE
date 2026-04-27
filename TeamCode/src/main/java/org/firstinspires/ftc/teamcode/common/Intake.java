package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private DcMotorEx intakeMotor = null;

    private Servo grabDexServoL = null;

    private Servo grabDexServoR = null;

    private Servo swingArmServo = null;

    private ColorRangeSensor ballSensorL = null;

    private ColorRangeSensor ballSensorR = null;

    static private double SWINGARM_LOW_POS = 0.8;

    static private double SWINGARM_UP_POS = 0.55;

    static private double SWINGARM_SHOOT_POS = 0.72;

    public enum IntakeState {
        ON(1.0),
        AUTO_SHOOT(0.65),
        OFF(0.0),
        REVERSE(-1.0);

        public final double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    public enum GrabdexerState {
        IN(0.987),
        TRANSFER(0.94),
        OUT(0.2),
        MID(0.6);

        public final double position;

        GrabdexerState(double position) {
            this.position = position;
        }
    }

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public IntakeState intakeState = IntakeState.OFF;

    public  GrabdexerState grabdexerState = GrabdexerState.IN;

    public Intake(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {
        intakeMotor = hMap.get(DcMotorEx.class, "intakeMotor");
        grabDexServoL = hMap.get(Servo.class, "grabDexServoL");
        grabDexServoR = hMap.get(Servo.class, "grabDexServoR");
        swingArmServo = hMap.get(Servo.class, "swingArmServo");
        ballSensorL = hMap.get(ColorRangeSensor.class, "ballSensorL");
        ballSensorR = hMap.get(ColorRangeSensor.class, "ballSensorR");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ballSensorL.setGain(8);
        ballSensorR.setGain(8);

        setGrabdexerState(grabdexerState);
        lowerSwingArm();
    }

    public void setIntakeState(IntakeState newState) {
        this.intakeState = newState;
        intakeMotor.setPower(intakeState.power);
    }

    public void setGrabdexerState(GrabdexerState newState) {
        this.grabdexerState = newState;
        grabDexServoL.setPosition(grabdexerState.position);
        grabDexServoR.setPosition(grabdexerState.position);
    }

    public void lowerSwingArm(){
        swingArmServo.setPosition(SWINGARM_LOW_POS);
    }

    public void raiseSwingArm(){
        swingArmServo.setPosition(SWINGARM_UP_POS);
    }

    public void shootPosSwingArm(){
        swingArmServo.setPosition(SWINGARM_SHOOT_POS);
    }

    public DetectedColor getDectectedColor(Telemetry telemetry){
        NormalizedRGBA colors = ballSensorL.getNormalizedColors();

        float normRed, normGreen, normBlue;

        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        telemetry.addData("Red Value:", normRed);
        telemetry.addData("Green Value:", normGreen);
        telemetry.addData("Blur Value:", normBlue);
        return DetectedColor.UNKNOWN;
    }

    public boolean isBallInGrabdexer(){
        double threshold = 64;
        double distance1 = ballSensorL.getDistance(DistanceUnit.MM);
        double distance2 = ballSensorR.getDistance(DistanceUnit.MM);
        return distance1 < threshold || distance2 < threshold;
    }
}
