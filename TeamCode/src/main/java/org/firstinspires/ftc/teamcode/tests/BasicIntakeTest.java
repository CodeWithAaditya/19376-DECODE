package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.Intake;

@TeleOp(name = "Basic Intake Test")
public class BasicIntakeTest extends LinearOpMode {

    public Intake intake = null;

    @Override
    public void runOpMode(){
        intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            intake.setIntakeState(Intake.IntakeState.ON);
            if(intake.isBallInGrabdexer()){
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                sleep(2000);
                intake.setIntakeState(Intake.IntakeState.OFF);
                sleep(2000);
                intake.setGrabdexerState(Intake.GrabdexerState.OUT);
            }

//            intake.setGrabdexerState(Intake.GrabdexerState.IN);
//            sleep(1000);
//            intake.setGrabdexerState(Intake.GrabdexerState.MID);
//            sleep(1000);
//            intake.setGrabdexerState(Intake.GrabdexerState.OUT);
//            sleep(1000);
//            intake.setGrabdexerState(Intake.GrabdexerState.MID);
//            sleep(1000);
            telemetry.update();
        }
    }
}
