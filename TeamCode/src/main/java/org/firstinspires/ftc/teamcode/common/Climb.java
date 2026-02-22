package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climb {
    private Servo ptoServoL, ptoServoR;

    public ColorRangeSensor colorSensor1, colorSensor2;

    private DcMotorEx driveBL, driveBR;

    private HardwareMap hMap;

    private static final double PTO_ENGAGE_POS = 0.11;
    private static final double PTO_EBRAKE_POS = 0.7;

    private static final double CLIMB_TARGET = 8192 * 3.3;

    public Climb(HardwareMap hardwareMap) {
        hMap = hardwareMap;

        ptoServoL = hardwareMap.get(Servo.class, "PTOL");
        ptoServoR = hardwareMap.get(Servo.class, "PTOR");

//        colorSensor1 = hardwareMap.get(ColorRangeSensor.class, "parkSensorL");
//        colorSensor2 = hardwareMap.get(ColorRangeSensor.class, "parkSensorR");

        eBrakePTO();
    }

    public void initClimb() {
        driveBL = hMap.get(DcMotorEx.class, "driveBL");
        driveBR = hMap.get(DcMotorEx.class, "driveBR");
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);

        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        engagePTO();
    }

    public void climbLoop() {
        if (getClimbPosition() <= CLIMB_TARGET) {
            driveBL.setPower(1.0);
            driveBR.setPower(1.0);
        } else {
            driveBL.setPower(0.3);
            driveBR.setPower(0.3);
        }
    }

    public int getClimbPosition() {
        return -driveBL.getCurrentPosition();
    }

    public void engagePTO() {
        ptoServoL.setPosition(PTO_ENGAGE_POS);
        ptoServoR.setPosition(PTO_EBRAKE_POS - 0.05);
    }

    public void eBrakePTO() {
        ptoServoL.setPosition(PTO_EBRAKE_POS);
        ptoServoR.setPosition(PTO_ENGAGE_POS);
    }
}