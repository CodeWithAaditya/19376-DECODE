package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private DcMotorEx intakeMotor = null;

    private Servo linkageServo = null;

    private ColorRangeSensor colorRangeSensor = null;

    public enum IntakeState {
        ON(1.0),
        OFF(0.0),
        REVERSE(-1.0);

        public final double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    public enum GrabdexerState {
        IN(0.52),
        OUT(0.18),
        MID(0.38);

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
        linkageServo = hMap.get(Servo.class, "linkageServo");
        colorRangeSensor = hMap.get(ColorRangeSensor.class, "colorSensor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorRangeSensor.setGain(8);

        setGrabdexerState(grabdexerState);
    }

    public void setIntakeState(IntakeState newState) {
        this.intakeState = newState;
        intakeMotor.setPower(intakeState.power);
    }

    public void setGrabdexerState(GrabdexerState newState) {
        this.grabdexerState = newState;
        linkageServo.setPosition(grabdexerState.position);
    }

    public DetectedColor getDectectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorRangeSensor.getNormalizedColors();

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
        double threshold = 70;
        double distance = colorRangeSensor.getDistance(DistanceUnit.MM);
        return distance < threshold;
    }
}
