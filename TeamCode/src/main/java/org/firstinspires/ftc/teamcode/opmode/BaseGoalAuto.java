package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.common.drive.GoBildaPinpointDriver;

import java.util.Locale;


public abstract class BaseGoalAuto extends LinearOpMode {
    protected boolean alliance;
    protected abstract boolean getAlliance();

    DcMotor driveFL;
    DcMotor driveFR;
    DcMotor driveBL;
    DcMotor driveBR;
    ElapsedTime timer = null;

    Intake intake;
    Shooter shooter;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    double negate;

    double mainSpeed = 0.9;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_TARGET_1,
        SHOOT_PRELOAD,
        SHOOT_2ND,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        SHOOT_3RD,
        DRIVE_TO_TARGET_10,
        DRIVE_TO_TARGET_11,
        DRIVE_TO_TARGET_12,
        SHOOT_4TH,
        DONE
    }

    Pose2D TARGET_1;
    Pose2D TARGET_2;
    Pose2D TARGET_3;
    Pose2D TARGET_4;
    Pose2D TARGET_5;
    Pose2D TARGET_6;
    Pose2D TARGET_7;
    Pose2D TARGET_8;
    Pose2D TARGET_9;
    Pose2D TARGET_10;
    Pose2D TARGET_11;
    Pose2D TARGET_12;

    Pose2D goalPose;

    @Override
    public void runOpMode() {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        alliance = getAlliance();

        if(alliance){
            negate = -1.0;
        } else {
            negate = 1.0;
        }

        TARGET_1 = new Pose2D(DistanceUnit.MM,600, 900*negate,AngleUnit.DEGREES,-180*negate);
        TARGET_2 = new Pose2D(DistanceUnit.MM, 600, 1300*negate, AngleUnit.DEGREES, -180*negate);
        TARGET_3 = new Pose2D(DistanceUnit.MM,-315,1300*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_4 = new Pose2D(DistanceUnit.MM,100,1600*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_5 = new Pose2D(DistanceUnit.MM,-300,1600*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_6 = new Pose2D(DistanceUnit.MM,600, 900*negate,AngleUnit.DEGREES,-180*negate);
        TARGET_7 = new Pose2D(DistanceUnit.MM,600,1950*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_8 = new Pose2D(DistanceUnit.MM,-400,1950*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_9 = new Pose2D(DistanceUnit.MM,600,1400*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_10 = new Pose2D(DistanceUnit.MM,600,2550*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_11 = new Pose2D(DistanceUnit.MM,-400,2550*negate, AngleUnit.DEGREES,-180*negate);
        TARGET_12 = new Pose2D(DistanceUnit.MM,600,1400*negate, AngleUnit.DEGREES,-180*negate);

        if(alliance){
            goalPose  = new Pose2D(INCH, -20, 6.5, DEGREES, 0);
        } else {
            goalPose  = new Pose2D(INCH, -20, -6.5, DEGREES, 0);
        }

        timer = new ElapsedTime();

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(8, -165); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            switch (stateMachine){
                case WAITING_FOR_START:
                    shooter.setTurretAngle(35*negate);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), TARGET_1, mainSpeed, 0)){
                        intake.setIntakeState(Intake.IntakeState.ON);
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_PRELOAD;
                    }
                    break;
                case SHOOT_PRELOAD:
                    if(timer.time()>2){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, mainSpeed, 0)){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(intake.isBallInGrabdexer()){
                        intake.setGrabdexerState(Intake.GrabdexerState.MID);
                    }
                    if(nav.driveTo(odo.getPosition(), TARGET_3, 0.5, 0)){
                        intake.setIntakeState(Intake.IntakeState.OFF);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(), TARGET_4, mainSpeed, 0)){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(), TARGET_5, mainSpeed, 0)){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if(nav.driveTo(odo.getPosition(),TARGET_6,mainSpeed,0)){
                        intake.setGrabdexerState(Intake.GrabdexerState.IN);
                        intake.setIntakeState(Intake.IntakeState.ON);
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_2ND;
                    }
                    break;
                case SHOOT_2ND:
                    if(timer.time()>2){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if(nav.driveTo(odo.getPosition(),TARGET_7,mainSpeed,0)){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if(intake.isBallInGrabdexer()){
                        intake.setGrabdexerState(Intake.GrabdexerState.MID);
                    }
                    if(nav.driveTo(odo.getPosition(), TARGET_8, 0.55, 0)){
                        intake.setIntakeState(Intake.IntakeState.OFF);
                        shooter.setTurretAngle(40*negate);   //holds rest of auto
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if(nav.driveTo(odo.getPosition(), TARGET_9, mainSpeed, 0)){
                        intake.setGrabdexerState(Intake.GrabdexerState.IN);
                        intake.setIntakeState(Intake.IntakeState.ON);
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_3RD;
                    }
                    break;
                case SHOOT_3RD:
                    if(timer.time()>2){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if(nav.driveTo(odo.getPosition(),TARGET_10,mainSpeed,0)){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if(intake.isBallInGrabdexer()){
                        intake.setGrabdexerState(Intake.GrabdexerState.MID);
                    }
                    if(nav.driveTo(odo.getPosition(), TARGET_11, 0.55, 0)){
                        intake.setIntakeState(Intake.IntakeState.OFF);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if(nav.driveTo(odo.getPosition(), TARGET_12, mainSpeed, 0)){
                        intake.setGrabdexerState(Intake.GrabdexerState.IN);
                        intake.setIntakeState(Intake.IntakeState.ON);
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_4TH;
                    }
                    break;
                case SHOOT_4TH:
                    if(timer.time()>2){
                        intake.setIntakeState(Intake.IntakeState.OFF);
                        timer.reset();
                        stateMachine = StateMachine.DONE;
                    }
                    break;
            }

            driveFL.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            driveFR.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            driveBL.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            driveBR.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            double distance = shooter.distanceToGoal(odo.getPosition(), goalPose);

            double[] settings = shooter.getShooterSettingsFromDistance(distance);

            double flywheelVel = settings[0];
            double hoodPos = settings[1];

            shooter.setShooterVelocity(flywheelVel);
            shooter.setHoodServoPos(hoodPos);

            shooter.update();

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();
        }
    }}
