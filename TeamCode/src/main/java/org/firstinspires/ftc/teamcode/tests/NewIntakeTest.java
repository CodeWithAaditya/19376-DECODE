package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Intake;

@TeleOp(name = "NewIntakeTest")
public class NewIntakeTest extends LinearOpMode {

    public Intake intake = null;

    public CRServo intakeServo = null;

    @Override
    public void runOpMode(){

        intake = new Intake(hardwareMap);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            intake.lowerSwingArm();
            if(gamepad1.right_trigger>0.5){
                intake.setIntakeState(Intake.IntakeState.ON);
                intakeServo.setPower(1.0);
            } else {
                intake.setIntakeState(Intake.IntakeState.OFF);
                intakeServo.setPower(0.0);
            }
        }
    }
}
