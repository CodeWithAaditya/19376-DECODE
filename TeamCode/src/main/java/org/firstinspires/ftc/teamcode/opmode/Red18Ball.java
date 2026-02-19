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

@Autonomous(name = "Red18Ball", group = "comp")
public class Red18Ball extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    public static class Paths {

        public PathChain preload;
        public PathChain spike1;
        public PathChain shootSpike1;
        public PathChain gate1;
        public PathChain shootGate1;
        public PathChain gate2;
        public PathChain shootGate2;
        public PathChain gate3;
        public PathChain shootGate3;
        public PathChain spike2;
        public PathChain shootSpike2;
        public PathChain spike3;
        public PathChain shootSpike3;

        public Paths(Follower follower) {

            preload = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(113.000, 135.000),
                                    new Pose(91.000, 93.000),
                                    new Pose(88.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-40))
                    .build();

            spike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 80.000),
                                    new Pose(86.000, 58.000),
                                    new Pose(106.000, 60.000),
                                    new Pose(131.000, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(0))
                    .build();

            shootSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.000, 59.000),
                                    new Pose(95.000, 57.000),
                                    new Pose(88.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            gate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 80.000),
                                    new Pose(98.000, 65.000),
                                    new Pose(130.000, 59.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(30))
                    .build();

            shootGate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 59.000),
                                    new Pose(98.000, 65.000),
                                    new Pose(88.000, 80.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(30))
                    .build();

            gate2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 80.000),
                                    new Pose(98.000, 65.000),
                                    new Pose(130.000, 59.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(30))
                    .build();

            shootGate2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 59.000),
                                    new Pose(98.000, 65.000),
                                    new Pose(88.000, 80.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(30))
                    .build();

            gate3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 80.000),
                                    new Pose(98.000, 65.000),
                                    new Pose(130.000, 59.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(30))
                    .build();

            shootGate3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 59.000),
                                    new Pose(91.000, 70.000),
                                    new Pose(87.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            spike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 84.000),
                                    new Pose(123.000, 83.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shootSpike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.000, 83.000),
                                    new Pose(80.000, 97.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            spike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(80.000, 97.000),
                                    new Pose(87.000, 50.000),
                                    new Pose(86.000, 34.000),
                                    new Pose(103.000, 34.000),
                                    new Pose(129.000, 35.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shootSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.000, 35.000),
                                    new Pose(106.000, 87.000),
                                    new Pose(90.000, 100.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    private ElapsedTime pathTimer, actionTimer;
    private int nextState = 0;
    private double actionDelay = 0;

    public Intake intake;
    public Shooter shooter;

    public static final double GATE_DELAY = 2;
    public static final double SHOOT_DELAY = 0.6;

    public Pose goalPose = new Pose(144, 144, 0);

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(112, 135, Math.toRadians(-90)));

        paths = new Paths(follower);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        initRobot();

        setPathState(2);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        autonomousPathUpdate();

//        double[] shooterSettings = shooter.SOTMTurretAngle(follower.getPose(), goalPose, follower.getVelocity());
//
//        shooter.setTurretAngle(shooterSettings[0]);
//        shooter.setShooterVelocity(shooterSettings[1]);
//        shooter.setHoodServoPos(shooterSettings[2]);

        double[] shooterSettings = shooter.getShooterSettingsFromDistance(shooter.distanceToGoal(follower.getPose(), goalPose));

        shooter.setShooterVelocity(shooterSettings[0]);
        shooter.setHoodServoPos(shooterSettings[1]);
        shooter.setTurretAngle(shooter.autoAimTurretAngle(follower.getPose(), goalPose));

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

    //case 0 is shoot
    //case 2 is shooter delay

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.setIntakeState(Intake.IntakeState.ON);
                intake.setGrabdexerState(Intake.GrabdexerState.IN);
                intake.shootPosSwingArm();
                //delay
                setPathState(1);
                break;
            case 1:
                if (delayCheck()) {
                    setPathState(nextState);
                }
                break;
            case -2:
                if (actionTimer.seconds()>0.2) {
                    setPathState(0);
                }
                break;
            case 2:
                follower.followPath(paths.preload);
                setPathState(3);
                break;
            case 3:
                if(pathCheck()){
                    shoot(4, SHOOT_DELAY);
                }
                break;
            case 4:
                follower.followPath(paths.spike1);
                setPathState(5);
                break;
            case 5:
                intake();
                if(pathCheck()){
                    setPathState(6);
                }
                break;
            case 6:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                follower.followPath(paths.shootSpike1);
                setPathState(7);
                break;
            case 7:
                if(pathCheck()){
                    shoot(8, SHOOT_DELAY);
                }
                break;
            case 8:
                follower.followPath(paths.gate1);
                setPathState(9);
                break;
            case 9:
                intake();
                if(pathCheck()){
                    actionDelay = GATE_DELAY-0.5;
                    setPathState(10);
                }
                break;
            case 10:
                intake();
                if(delayCheck()){
                    setPathState(11);
                }
                break;
            case 11:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                follower.followPath(paths.shootGate1);
                setPathState(12);
                break;
            case 12:
                if(pathCheck()){
                    shoot(13, SHOOT_DELAY);
                }
                break;
            case 13:
                follower.followPath(paths.gate2);
                setPathState(14);
                break;
            case 14:
                intake();
                if(pathCheck()){
                    actionDelay = GATE_DELAY;
                    setPathState(15);
                }
                break;
            case 15:
                intake();
                if(delayCheck()){
                    setPathState(16);
                }
                break;
            case 16:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                follower.followPath(paths.shootGate2);
                setPathState(17);
                break;
            case 17:
                if(pathCheck()){
                    shoot(18, SHOOT_DELAY);
                }
                break;
            case 18:
                follower.followPath(paths.gate3);
                setPathState(19);
                break;
            case 19:
                intake();
                if(pathCheck()){
                    actionDelay = GATE_DELAY;
                    setPathState(20);
                }
                break;
            case 20:
                intake();
                if(delayCheck()){
                    setPathState(21);
                }
                break;
            case 21:
                intake.setGrabdexerState(Intake.GrabdexerState.MID);
                follower.followPath(paths.shootGate3);
                setPathState(22);
                break;
            case 22:
                if(pathCheck()){
                    shoot(23, SHOOT_DELAY);
                }
                break;
            case 23:
                follower.followPath(paths.spike2);
                setPathState(24);
                break;
            case 24:
                intake();
                if(pathCheck()){
                    setPathState(25);
                }
                break;
            case 25:
                follower.followPath(paths.shootSpike2);
                setPathState(26);
                break;
            case 26:
                if(pathCheck()){
                    shoot(-1, SHOOT_DELAY);
                }
                break;
//            case 27:
//                follower.followPath(paths.spike3);
//                setPathState(28);
//                break;
//            case 28:
//                intake();
//                if(pathCheck()){
//                    setPathState(29);
//                }
//                break;
//            case 29:
//                follower.followPath(paths.shootSpike3);
//                setPathState(30);
//            case 30:
//                if(pathCheck()){
//                    shoot(-1, SHOOT_DELAY);
//                }
//                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; actionTimer.reset(); pathTimer.reset(); }

    public void shoot(int targetState, double delayTime){
        nextState = targetState;
        actionDelay = delayTime;
        setPathState(-2);
    }

    public boolean pathCheck(){
        return !follower.isBusy();
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
