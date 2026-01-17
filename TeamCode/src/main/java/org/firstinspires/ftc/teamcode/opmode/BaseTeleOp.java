package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrive;

public abstract class BaseTeleOp extends LinearOpMode {
    protected boolean alliance;
    protected abstract boolean getAlliance();

    public MecanumDrive drive = null;
    public Intake intake = null;
    public Shooter shooter = null;

    Pose2D goalPose;

    public enum RobotState {
        IDLE,
        INTAKE,
        AIM,
        SHOOT,
        MANUAL_OVERRIDE,
    }

    public BaseTeleOp.RobotState robotState = BaseTeleOp.RobotState.IDLE;

    @Override
    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        alliance = getAlliance();

        if(alliance){
            goalPose  = new Pose2D(INCH, 72, 72, DEGREES, 0);
        } else {
            goalPose  = new Pose2D(INCH, 72, -72, DEGREES, 0);
        }

        drive.odo.resetPosAndIMU();
        drive.odo.update();

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
                robotState = BaseTeleOp.RobotState.IDLE;
            }
            if(gamepad1.right_trigger>0.5){
                robotState = BaseTeleOp.RobotState.INTAKE;
            }
            if(gamepad1.left_bumper){
                robotState = BaseTeleOp.RobotState.SHOOT;
            }
            if(gamepad1.y){
                shooter.setTurretAngle(0);
            }
            if(gamepad1.b){
                drive.odo.resetPosAndIMU();
            }
            if(gamepad1.left_trigger>0.5){
                intake.setIntakeState(Intake.IntakeState.OFF);
            }

            if(gamepad1.backWasPressed()){
                shooter.trimTurretOffset(-5);
            }
            if(gamepad1.startWasPressed()){
                shooter.trimTurretOffset(5);
            }

            double turretTarget = shooter.autoAimTurretAngle(drive.getPose(), goalPose);
            shooter.setTurretAngle(turretTarget);

            double distance = shooter.distanceToGoal(drive.getPose(), goalPose);

            double[] settings = shooter.getShooterSettingsFromDistance(distance);

            double flywheelVel = settings[0];
            double hoodPos = settings[1];

            shooter.setShooterVelocity(flywheelVel);
            shooter.setHoodServoPos(hoodPos);

            shooter.update();

            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x*0.6);

            telemetry.addData("robot pos x:", drive.getPose().getX(INCH));
            telemetry.addData("robot pos y:", drive.getPose().getX(INCH));
            telemetry.addData("robot pos h:", drive.getPose().getHeading(DEGREES));
            telemetry.update();
        }
    }
}
