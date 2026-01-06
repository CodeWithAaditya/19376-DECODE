package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive;

@TeleOp(name="DrivetrainTest")
public class DrivetrainTest extends LinearOpMode {
    public MecanumDrive drive = null;
    public Intake intake = null;
    public Shooter shooter = null;

    private DcMotor motor;
    private PIDLoop1 pid = new PIDLoop1();

    // PID constants (TUNE)
    private static final double kP = 0.002;
    private static final double kD = 0.0002;
    private static final double ACCEL_LIMIT = 2.0; // power/sec

    private double targetPosition = 0.0;

    public enum RobotState {
        IDLE,
        INTAKE,
        AIM,
        SHOOT,
        MANUAL_OVERRIDE,
    }

    public RobotState robotState = RobotState.IDLE;

    @Override
    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        motor = hardwareMap.get(DcMotor.class, "hangMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPosition = 0.0;

        drive.setManualMode();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            switch (robotState){
                case IDLE:
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    break;
                case INTAKE:
                    if(gamepad1.right_trigger>0.5){
                        intake.setIntakeState(Intake.IntakeState.ON);
                    }else {
                        intake.setIntakeState(Intake.IntakeState.OFF);
                    }

                    if(intake.isBallInGrabdexer()){
                        intake.setGrabdexerState(Intake.GrabdexerState.MID);
                    }
                    break;
                case AIM:
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    shooter.setShooterVelocity(1700);
                    //auto aim code
                    break;
                case SHOOT:
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    intake.setIntakeState(Intake.IntakeState.ON);
                    //still auto aim
                    //elapsed time and then go to IDLE
            }

            if(gamepad1.a){
                robotState = RobotState.IDLE;
            }
            if(gamepad1.right_trigger>0.5){
                robotState = RobotState.INTAKE;
            }
            if(gamepad1.right_bumper && intake.isBallInGrabdexer()) {
                robotState = RobotState.AIM;
            }
            if(gamepad1.left_bumper){
                robotState = RobotState.SHOOT;
            }
            if(gamepad1.y){
                shooter.setShooterVelocity(0);
            }
            if(gamepad1.back){
                targetPosition = -9850;
            }
            if(gamepad1.dpad_up){
                shooter.setShooterVelocity(2800);
                shooter.setHoodServoPos(0.2);
            }
            if(gamepad1.dpad_down){
                shooter.setShooterVelocity(1700);
                shooter.setHoodServoPos(0.15);
            }
            if(gamepad1.dpad_left){
                shooter.setTurretAngle(40);
            }
            if(gamepad1.dpad_right){
                shooter.setTurretAngle(-40);
            }
            if(gamepad1.x){
                shooter.setTurretAngle(0);
            }
            if(gamepad1.b){
                drive.odo.resetPosAndIMU();
            }
            if(gamepad1.left_trigger>0.5){
                intake.setIntakeState(Intake.IntakeState.OFF);
            }
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x*0.6);
            shooter.update();

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
}

class PIDLoop1 {
    private double previousError;
    private double previousTime;
    private double previousOutput;

    private double errorR;

    public double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime) {
        double p = error * pGain;
        double cycleTime = currentTime - previousTime;
        double d = dGain * (previousError - error) / (cycleTime);
        double output = p + d;
        double dV = cycleTime * accel;

        double max = Math.abs(output);
        if (max > 1.0) {
            output /= max;
        }

        if ((output - previousOutput) > dV) {
            output = previousOutput + dV;
        } else if ((output - previousOutput) < -dV) {
            output = previousOutput - dV;
        }

        previousOutput = output;
        previousError = error;
        previousTime = currentTime;

        errorR = error;

        return output;

    }
}