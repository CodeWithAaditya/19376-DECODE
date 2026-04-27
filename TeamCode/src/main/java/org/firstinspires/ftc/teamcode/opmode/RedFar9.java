package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.Constants;

@Autonomous(name = "RedFar9", group = "comp")
public class RedFar9 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    public static class Paths {
        public PathChain hpZone;
        public PathChain hpBackup;
        public PathChain hpZone2;
        public PathChain shootHp;
        public PathChain intakeSpike;
        public PathChain shootSpike;
        public PathChain leave;

        public Paths(Follower follower) {
            hpZone = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 8.000),
                                    new Pose(88.000, 45.000),
                                    new Pose(110.000, 9.000),
                                    new Pose(120.000, 10.000),
                                    new Pose(133.000, 10.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            hpBackup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.000, 10.000),

                                    new Pose(122.000, 10.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            hpZone2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.000, 10.000),

                                    new Pose(133.000, 10.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootHp = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.000, 10.000),
                                    new Pose(95.000, 21.000),
                                    new Pose(86.000, 21.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeSpike = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 21.000),
                                    new Pose(94.000, 35.000),
                                    new Pose(107.000, 36.000),
                                    new Pose(128.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootSpike = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.000, 36.000),
                                    new Pose(104.000, 21.000),
                                    new Pose(86.000, 21.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 21.000),

                                    new Pose(104.000, 21.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }

    private ElapsedTime pathTimer, actionTimer;
    private int nextState = 0;
    private double actionDelay = 0;

    public Intake intake;
    public Shooter shooter;

    public static final double PRELOAD_ANGLE = -20;
    public static final double PRELOAD_FLYWHEEL = 1900;

    public static final double CYCLE_ANGLE = 67;
    public static final double CYCLE_FLYWHEEL = 1850;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        initRobot();

        setPathState(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void initRobot(){
        intake.setIntakeState(Intake.IntakeState.OFF);
        intake.setGrabdexerState(Intake.GrabdexerState.TRANSFER);
        intake.lowerSwingArm();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(delayCheck()){
                    setPathState(nextState);
                }
                break;
            case 1:
                shooter.setTurretAngle(PRELOAD_ANGLE);
                shooter.setShooterVelocity(PRELOAD_FLYWHEEL);
                delay(-2, 3.0);
                break;
            case -2:
                intake.setGrabdexerState(Intake.GrabdexerState.IN);
                intake.shootPosSwingArm();
                intake.setIntakeState(Intake.IntakeState.ON);
                delay(2, 1.0);
                break;
            case 2:
                intake();
                follower.followPath(paths.hpZone);
                setPathState(3);
            case 3:
                intake();
                if(pathCheck()){
                    delay(4, 1.0);
                }
                break;
            case 4:
                intake();
                follower.followPath(paths.hpBackup);
                setPathState(5);
                break;
            case 5:
                intake();
                if(pathCheck()){
                    setPathState(6);
                }
                break;
            case 6:
                intake();
                follower.followPath(paths.hpZone2);
                setPathState(7);
            case 7:
                intake();
                if(pathCheck()){
                    delay(8, 1.0);
                }
                break;
            case 8:
                //maybe add intake off here
                shooter.setTurretAngle(CYCLE_ANGLE);
                shooter.setShooterVelocity(CYCLE_FLYWHEEL);
                follower.followPath(paths.shootHp);
                setPathState(9);
                break;
            case 9:
                if(pathCheck()){
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    intake.shootPosSwingArm();
                    intake.setIntakeState(Intake.IntakeState.ON);
                    delay(10, 1.0);
                }
                break;
            case 10:
                intake();
                follower.followPath(paths.intakeSpike);
                setPathState(11);
                break;
            case 11:
                intake();
                if(pathCheck()){
                    setPathState(12);
                }
                break;
            case 12:
                //maybe add turn off intake here
                shooter.setTurretAngle(CYCLE_ANGLE+2);
                shooter.setShooterVelocity(CYCLE_FLYWHEEL);
                follower.followPath(paths.shootSpike);
                setPathState(13);
                break;
            case 13:
                if(pathCheck()){
                    intake.setGrabdexerState(Intake.GrabdexerState.IN);
                    intake.shootPosSwingArm();
                    intake.setIntakeState(Intake.IntakeState.ON);
                    delay(14, 1.0);
                }
                break;
            case 14:
                shooter.setShooterVelocity(0);
                shooter.setTurretAngle(0);
                intake.setIntakeState(Intake.IntakeState.OFF);
                follower.followPath(paths.leave);
                setPathState(15);
                break;
            case 15:
                if(pathCheck()){
                    setPathState(-1);
                }
                break;
            case -1:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; actionTimer.reset(); pathTimer.reset(); }

    public boolean pathCheck(){
        return !follower.isBusy();
    }

    public void delay(int targetState, double time){
        nextState = targetState;
        actionDelay = time;
        setPathState(0);
    }

    public boolean delayCheck(){
        return actionTimer.seconds()>=actionDelay;
    }

    public void intake(){
        intake.setIntakeState(Intake.IntakeState.ON);
        if(intake.isBallInGrabdexer()){
            intake.setGrabdexerState(Intake.GrabdexerState.MID);
            intake.raiseSwingArm();
        }else {
            intake.setGrabdexerState(Intake.GrabdexerState.TRANSFER);
            intake.lowerSwingArm();
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}
