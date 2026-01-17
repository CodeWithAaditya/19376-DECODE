package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
@Disabled
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo servo = hardwareMap.get(Servo.class, "turretServoR");
        AnalogInput turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        waitForStart();

        while (opModeIsActive()) {

            // Move to 0.0
//            servo.setPosition(0.0);
//            telemetry.addData("Servo Position", 0.0);
//            telemetry.addData("Turret Value:", turretEncoder.getVoltage());
//            telemetry.update();
//            sleep(800);

            // Move to 0.5
            servo.setPosition(0.5);
            telemetry.addData("Servo Position", 0.5);
            telemetry.addData("Turret Value:", turretEncoder.getVoltage());
            telemetry.update();
            sleep(800);

            // Move to 1.0
//            servo.setPosition(1.0);
//            telemetry.addData("Servo Position", 1.0);
//            telemetry.addData("Turret Value:", turretEncoder.getVoltage());
//            telemetry.update();
//            sleep(800);

            // Back to 0.5
            servo.setPosition(0.5);
            telemetry.addData("Servo Position", 0.5);
            telemetry.addData("Turret Value:", turretEncoder.getVoltage());
            telemetry.update();
            sleep(800);
        }
    }
}
