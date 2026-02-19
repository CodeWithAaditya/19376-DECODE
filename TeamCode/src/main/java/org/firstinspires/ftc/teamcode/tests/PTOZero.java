package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive;

@Disabled
@TeleOp(name = "PTOZero")
public class PTOZero extends LinearOpMode {

    public MecanumDrive drive;

    @Override
    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap);
         waitForStart();
         while (!isStopRequested() && opModeIsActive()){
             if(gamepad1.aWasPressed()) {
                 drive.engagePTO();
             }
             if(gamepad1.bWasPressed()) {
                 drive.eBrakePTO();
             }
             drive.driveBL.setPower(-gamepad1.left_stick_y);
             drive.driveBR.setPower(-gamepad1.left_stick_y);
         }
     }
}
