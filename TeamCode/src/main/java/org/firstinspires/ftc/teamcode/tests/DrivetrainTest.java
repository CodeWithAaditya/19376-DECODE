package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

        drive.odo.resetPosAndIMU();
        drive.odo.update();

        drive.setManualMode();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.odo.update();
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
                case SHOOT:
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    intake.setIntakeState(Intake.IntakeState.ON);
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
                shooter.setShooterVelocity(2050);
                shooter.setHoodServoPos(0.3);
            }
            if(gamepad1.dpad_down){
                shooter.setShooterVelocity(1500);
                shooter.setHoodServoPos(0.2);
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

            Pose2D goalPose  = new Pose2D(INCH, 72, 72, DEGREES, 0); // example goal

            double turretTarget = shooter.autoAimTurretAngle(drive.getPose(), goalPose);

            double distance = shooter.distanceToGoal(drive.getPose(), goalPose);

            double[] settings = shooter.getShooterSettingsFromDistance(distance);

            double flywheelVel = settings[0];
            double hoodPos = settings[1];

            shooter.setShooterVelocity(flywheelVel);
            shooter.setHoodServoPos(hoodPos);
            shooter.setTurretAngle(turretTarget);

            shooter.update();

            telemetry.addData("robot pos x:", drive.getPose().getX(INCH));
            telemetry.addData("robot pos y:", drive.getPose().getX(INCH));
            telemetry.addData("robot pos h:", drive.getPose().getHeading(DEGREES));
            telemetry.addData("Turret target", "%.1f", turretTarget);
            telemetry.addData("Robot heading deg", "%.1f", drive.getPose().getHeading(DEGREES));
            telemetry.addData("Field angle deg", "%.1f", Math.toDegrees(Math.atan2(72 - drive.getPose().getY(INCH), 72 - drive.getPose().getX(INCH))));
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