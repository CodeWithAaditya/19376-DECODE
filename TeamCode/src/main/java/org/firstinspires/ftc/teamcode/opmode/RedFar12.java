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

@Autonomous(name = "RedFar12", group = "comp")
public class RedFar12 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    public static class Paths {

        public PathChain preload;
        public PathChain spike1;
        public PathChain spike1Back;
        public PathChain spike1Fwd;
        public PathChain shootSpike1;
        public PathChain row2;
        public PathChain shootRow2;
        public PathChain spike2;
        public PathChain spike2Back;
        public PathChain spike2Fwd;
        public PathChain shootSpike2;
        public PathChain leave;

        public Paths(Follower follower) {

            preload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.000, 10.000),
                                    new Pose(86.000, 22.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            spike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 22.000),
                                    new Pose(110.000, 10.000),
                                    new Pose(131.000, 10.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            spike1Back = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.000, 10.000),
                                    new Pose(120.000, 10.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            spike1Fwd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.000, 10.000),
                                    new Pose(131.000, 10.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shootSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.000, 10.000),
                                    new Pose(111.000, 23.000),
                                    new Pose(86.000, 22.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            row2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 22.000),
                                    new Pose(102.000, 37.000),
                                    new Pose(129.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shootRow2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.000, 36.000),
                                    new Pose(86.000, 22.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            spike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.000, 22.000),
                                    new Pose(115.000, 9.000),
                                    new Pose(131.000, 16.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            spike2Back = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.000, 16.000),
                                    new Pose(126.000, 14.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            spike2Fwd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.000, 14.000),
                                    new Pose(131.000, 16.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shootSpike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.000, 16.000),
                                    new Pose(86.000, 22.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 22.000),
                                    new Pose(109.000, 23.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    private ElapsedTime pathTimer, actionTimer;

    public Intake intake;
    public Shooter shooter;

    public static final double SPINUP_DELAY  = 3.0;  // wait for shooter to spin up
    public static final double COLLECT_DELAY = 1.0;  // wait at HP zone for balls
    public static final double SHOOT_DELAY   = 1.2;  // shoot duration at each position
    public static final double FAR_TURRET_OFFSET_DEG = +3;

    private int nextState = 0;

    public Pose goalPose = new Pose(144, 144, 0);

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer  = new ElapsedTime();
        actionTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        try {
            follower.getPoseTracker().resetIMU();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        paths = new Paths(follower);

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        initRobot();
        setPathState(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start(){
        follower.setStartingPose(new Pose(89, 10, Math.toRadians(0)));
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        autonomousPathUpdate();

        double distance = shooter.distanceToGoal(follower.getPose(), goalPose);
        shooter.setTurretAngle(
                shooter.autoAimTurretAngle(follower.getPose(), goalPose) + FAR_TURRET_OFFSET_DEG);
        shooter.setShooterVelocity(shooter.getShooterSettingsFromDistance(distance)[0]);

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public void initRobot() {
        intake.setIntakeState(Intake.IntakeState.OFF);
        intake.setGrabdexerState(Intake.GrabdexerState.TRANSFER);
        intake.lowerSwingArm();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── Collect delay ─────────────────────────────────────────
            case 0:
                if (actionTimer.seconds() >= COLLECT_DELAY) {
                    setPathState(nextState);
                }
                break;

            // ── Shoot delay ───────────────────────────────────────────
            case -2:
                intake.setIntakeState(Intake.IntakeState.ON);
                intake.setGrabdexerState(Intake.GrabdexerState.IN);
                intake.shootPosSwingArm();
                if (actionTimer.seconds() >= SHOOT_DELAY) {
                    setPathState(nextState);
                }
                break;

            // ── Preload ───────────────────────────────────────────────
            case 1:
                follower.followPath(paths.preload);
                setPathState(2);
                break;
            case 2:
                if (pathCheck()) {
                    setPathState(3);
                }
                break;
            case 3:
                // spinup delay after arriving at shoot position
                if (actionTimer.seconds() >= SPINUP_DELAY) {
                    shoot(4);
                }
                break;

            // ── Spike1 (HP row 1) ─────────────────────────────────────
            case 4:
                follower.followPath(paths.spike1);
                setPathState(5);
                break;
            case 5:
                intake();
                if (pathCheck()) {
                    nextState = 6;
                    setPathState(0);
                }
                break;
            case 6:
                follower.followPath(paths.spike1Back);
                setPathState(7);
                break;
            case 7:
                intake();
                if (pathCheck()) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(paths.spike1Fwd);
                setPathState(9);
                break;
            case 9:
                intake();
                if (pathCheck()) {
                    setPathState(10);
                }
                break;
            case 10:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                intake.setIntakeState(Intake.IntakeState.OFF);
                follower.followPath(paths.shootSpike1);
                setPathState(11);
                break;
            case 11:
                if (pathCheck()) {
                    shoot(12);
                }
                break;

            // ── Row 2 (HP row 2) ──────────────────────────────────────
            case 12:
                follower.followPath(paths.row2);
                setPathState(13);
                break;
            case 13:
                intake();
                if (pathCheck()) {
                    nextState = 14;
                    setPathState(0);
                }
                break;
            case 14:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                intake.setIntakeState(Intake.IntakeState.OFF);
                follower.followPath(paths.shootRow2);
                setPathState(15);
                break;
            case 15:
                if (pathCheck()) {
                    shoot(16);
                }
                break;

            // ── Spike2 (HP row 1, second spot) ───────────────────────
            case 16:
                follower.followPath(paths.spike2);
                setPathState(17);
                break;
            case 17:
                intake();
                if (pathCheck()) {
                    nextState = 18;
                    setPathState(0);
                }
                break;
            case 18:
                follower.followPath(paths.spike2Back);
                setPathState(19);
                break;
            case 19:
                intake();
                if (pathCheck()) {
                    setPathState(20);
                }
                break;
            case 20:
                follower.followPath(paths.spike2Fwd);
                setPathState(21);
                break;
            case 21:
                intake();
                if (pathCheck()) {
                    setPathState(22);
                }
                break;
            case 22:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                intake.setIntakeState(Intake.IntakeState.OFF);
                follower.followPath(paths.shootSpike2);
                setPathState(23);
                break;
            case 23:
                if (pathCheck()) {
                    shoot(24);
                }
                break;

            // ── Leave ─────────────────────────────────────────────────
            case 24:
                follower.followPath(paths.leave);
                setPathState(25);
                break;
            case 25:
                if (pathCheck()) {
                    setPathState(-1);
                }
                break;

            case -1:
                requestOpModeStop();
                break;
        }
    }

    public void shoot(int targetState) {
        nextState = targetState;
        setPathState(-2);
    }

    public void setPathState(int pState) {
        pathState = pState;
        actionTimer.reset();
        pathTimer.reset();
    }

    public boolean pathCheck() {
        return !follower.isBusy();
    }

    public void intake() {
        intake.setIntakeState(Intake.IntakeState.ON);
        if (intake.isBallInGrabdexer()) {
            intake.setGrabdexerState(Intake.GrabdexerState.MID);
            intake.raiseSwingArm();
        } else {
            intake.setGrabdexerState(Intake.GrabdexerState.TRANSFER);
            intake.lowerSwingArm();
        }
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}
