package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Climb Test", group = "Test")
public class ClimbTest extends OpMode {

    private DcMotor motor;
    private PIDLoop pid = new PIDLoop();

    // PID constants (TUNE)
    private static final double kP = 0.003;
    private static final double kD = 0.0003;
    private static final double ACCEL_LIMIT = 2.0; // power/sec

    // Position scaling
    private static final double TICKS_PER_STICK = 40;

    private double targetPosition = 0.0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "hangMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPosition = 0.0;
    }

    @Override
    public void loop() {

        // Adjust target position with joystick
        double stick = -gamepad1.left_stick_y;
        targetPosition += stick * TICKS_PER_STICK;

        double currentPosition = motor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        double power = pid.calculateAxisPID(
                error,
                kP,
                kD,
                ACCEL_LIMIT,
                getRuntime()
        );

        motor.setPower(power);

        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("Current Pos", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.update();
    }
}

class PIDLoop{
    private double previousError;
    private double previousTime;
    private double previousOutput;

    private double errorR;

    public double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime){
        double p = error * pGain;
        double cycleTime = currentTime - previousTime;
        double d = dGain * (previousError - error) / (cycleTime);
        double output = p + d;
        double dV = cycleTime * accel;

        double max = Math.abs(output);
        if(max > 1.0){
            output /= max;
        }

        if((output - previousOutput) > dV){
            output = previousOutput + dV;
        } else if ((output - previousOutput) < -dV){
            output = previousOutput - dV;
        }

        previousOutput = output;
        previousError  = error;
        previousTime   = currentTime;

        errorR = error;

        return output;
    }

}
