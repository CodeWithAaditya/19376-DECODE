package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Climb;
import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.Constants;

public abstract class BaseTeleOp extends LinearOpMode {
    protected boolean alliance;
    protected abstract boolean getAlliance();

    private Follower follower;
    public Intake intake;
    public Shooter shooter;
    public Climb climb;

    Pose goalPose;

    public enum RobotState {
        IDLE,
        INTAKE,
        SHOOT,
        CLIMB
    }

    public BaseTeleOp.RobotState robotState = BaseTeleOp.RobotState.IDLE;

    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        climb = new Climb(hardwareMap);

        alliance = getAlliance();

        if(alliance){
            goalPose  = new Pose(0, 144, 0);
        } else {
            goalPose  = new Pose(144, 144, 0);
        }

        follower.startTeleOpDrive();

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
                        intake.raiseSwingArm();
                    }else{
                        intake.lowerSwingArm();
                        intake.setGrabdexerState(Intake.GrabdexerState.TRANSFER);
                    }
                    break;
                case SHOOT:
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    intake.setIntakeState(Intake.IntakeState.ON);
                    intake.shootPosSwingArm();
                    break;
                case CLIMB:
                    shooter.setTurretManual(0.0);
                    shooter.setHoodServoPos(0);
                    shooter.setShooterVelocity(0);
                    intake.setIntakeState(Intake.IntakeState.OFF);
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    climb.climbLoop();
                    break;
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
            if(gamepad1.bWasPressed()){
                if(alliance){
                    follower.setPose(new Pose(27, 132, Math.toRadians(-40)));
                } else {
                    follower.setPose(new Pose(117, 132, Math.toRadians(-140)));
                }
            }
            if(gamepad1.left_trigger>0.5){
                intake.setIntakeState(Intake.IntakeState.REVERSE);
            }
            if(gamepad1.backWasPressed()){
                shooter.trimTurretOffset(-2);
            }
            if(gamepad1.startWasPressed()){
                shooter.trimTurretOffset(2);
            }
            if(gamepad1.guideWasPressed()){
                climb.initClimb();
                robotState = RobotState.CLIMB;
            }
            if(robotState != RobotState.CLIMB) {
                double[] shooterSettings = shooter.SOTMTurretAngle(follower.getPose(), goalPose, follower.getVelocity());

                shooter.setTurretAngle(shooterSettings[0]);
                shooter.setShooterVelocity(shooterSettings[1]);
                shooter.setHoodServoPos(shooterSettings[2]);
            }

            if(robotState!=RobotState.CLIMB) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x * 0.6,
                        true
                );
            }

            shooter.update();
            follower.update();

            telemetry.addData("robot pos x:", follower.getPose().getX());
            telemetry.addData("robot pos y:", follower.getPose().getY());
            telemetry.addData("robot pos h:", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
