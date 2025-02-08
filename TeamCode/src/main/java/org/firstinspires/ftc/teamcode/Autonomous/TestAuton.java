package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.CLOSE_CLAW;
import static org.firstinspires.ftc.teamcode.RobotConstants.GRAB_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.HIGH_CHAMBER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.OPEN_CLAW;
import static org.firstinspires.ftc.teamcode.RobotConstants.VERT_LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.VERT_LIFT_POSITION_TOLERANCE;
import static org.firstinspires.ftc.teamcode.RobotConstants.WRIST_UPSIDE_UP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="Test Auton")
public class TestAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private MotorEx lsUp, rsUp;
    private ServoEx leftSwivelServo, rightSwivelServo, wrist, claw;
    private final Pose startPose = new Pose(10, 60, Math.toRadians(0));
    private final Pose preload = new Pose(36, 76, Math.toRadians(0)); //1
    private final Pose path1 = new Pose(36, 48, Math.toRadians(0)); //2
    private final Pose path2 = new Pose(66, 24, Math.toRadians(180)); //3
    private final Pose push1 = new Pose(12, 24, Math.toRadians(180)); //4
    private final Pose return1 = new Pose(66, 24, Math.toRadians(180)); //5 reverse
    private final Pose down1 = new Pose(66, 12, Math.toRadians(180)); //6
    private final Pose push2 = new Pose(12, 12, Math.toRadians(180)); //7
    private final Pose return2 = new Pose(66, 12, Math.toRadians(180)); //8 reverse
    private final Pose down2 = new Pose(66, 8.5, Math.toRadians(180)); //9
    private final Pose push3 = new Pose(12, 8.5, Math.toRadians(180)); //10
    private final Pose reload = new Pose(8 ,24, Math.toRadians(180)); //11
    private final Pose pickup1 = new Pose(36, 73, Math.toRadians(0)); //12
    private final Pose pickup2 = new Pose(36, 70, Math.toRadians(0)); //14
    private final Pose pickup3 = new Pose(36, 67, Math.toRadians(0)); //16
    private final Pose pickup4 = new Pose(36, 64, Math.toRadians(0)); //18
    private final Pose park1 = new Pose(36, 35, Math.toRadians(90)); //19
    private final Pose park2 = new Pose(60, 35, Math.toRadians(90)); //20
    private final Pose park3 = new Pose(60, 50, Math.toRadians(90)); //21
    private Path scorePreload;
    private PathChain positionToPush, pushSample1, pushSample2, pushSample3, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, grabPickup4, scorePickup4, park;
    private int vSlidePos, hSlidePos, vSlideTargetPos, hSlideTargetPos;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private MultipleTelemetry telemetryA;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preload)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preload.getHeading());

        positionToPush = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preload), new Point(path1)))
                .setConstantHeadingInterpolation(path1.getHeading())
                .addPath(new BezierLine(new Point(path1), new Point(path2)))
                .setLinearHeadingInterpolation(preload.getHeading(), path2.getHeading())
                .build();

        pushSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(path2), new Point(push1)))
                .setConstantHeadingInterpolation(path2.getHeading())
                .build();

        pushSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push1), new Point(return1)))
                .setConstantHeadingInterpolation(push1.getHeading())
                .addPath(new BezierLine(new Point(return1), new Point(down1)))
                .setConstantHeadingInterpolation(return1.getHeading())
                .addPath(new BezierLine(new Point(down1), new Point(push2)))
                .setConstantHeadingInterpolation(down1.getHeading())
                .build();

        pushSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push2), new Point(return2)))
                .setConstantHeadingInterpolation(push2.getHeading())
                .addPath(new BezierLine(new Point(return2), new Point(down2)))
                .setConstantHeadingInterpolation(return2.getHeading())
                .addPath(new BezierLine(new Point(down2), new Point(push3)))
                .setConstantHeadingInterpolation(down2.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push3), new Point(reload)))
                .setConstantHeadingInterpolation(push3.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reload), new Point(pickup1)))
                .setLinearHeadingInterpolation(reload.getHeading(), pickup1.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1), new Point(reload)))
                .setLinearHeadingInterpolation(pickup1.getHeading(), reload.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reload), new Point(pickup2)))
                .setLinearHeadingInterpolation(reload.getHeading(), pickup2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(reload)))
                .setLinearHeadingInterpolation(pickup2.getHeading(), reload.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reload), new Point(pickup3)))
                .setLinearHeadingInterpolation(reload.getHeading(), pickup3.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3), new Point(reload)))
                .setLinearHeadingInterpolation(pickup3.getHeading(), reload.getHeading())
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reload), new Point(pickup4)))
                .setLinearHeadingInterpolation(reload.getHeading(), pickup4.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup4), new Point(park1)))
                .setLinearHeadingInterpolation(pickup4.getHeading(), park1.getHeading())
                .addPath(new BezierLine(new Point(park1), new Point(park2)))
                .setConstantHeadingInterpolation(park2.getHeading())
                .addPath(new BezierLine(new Point(park2), new Point(park3)))
                .setConstantHeadingInterpolation(park3.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                wrist.setPosition(WRIST_UPSIDE_UP);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // score preload code

                    follower.followPath(positionToPush, true);
                    setPathState(2);
                }
                break;

//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(pushSample1, true);
//                    setPathState(3);
//                }
//                break;

//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(pushSample2, true);
//                    setPathState(4);
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(pushSample3, true);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    //already set to grab height earlier
//                    follower.followPath(grabPickup1, true);
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    //grab pickup
//
//                    if (claw.getPosition() == CLOSE_CLAW) {
//                        follower.followPath(scorePickup1, true);
//                    }
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    //score pickup
//
//                    if (claw.getPosition() == OPEN_CLAW) {
//                        follower.followPath(grabPickup2, true);
//                    }
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    //grab pickup
//
//                    if (claw.getPosition() == CLOSE_CLAW) {
//                        follower.followPath(scorePickup2);
//                    }
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                if (!follower.isBusy()) {
//                    //score pickup
//
//                    if (claw.getPosition() == OPEN_CLAW) {
//                        follower.followPath(grabPickup3);
//                    }
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    //grab pickup
//
//                    if (claw.getPosition() == CLOSE_CLAW) {
//                        follower.followPath(scorePickup3);
//                    }
//                    setPathState(11);
//                }
//                break;
//
//            case 11:
//                if (!follower.isBusy()) {
//                    //score pickup
//
//                    if (claw.getPosition() == OPEN_CLAW) {
//                        follower.followPath(grabPickup4);
//                    }
//                    setPathState(12);
//                }
//                break;
//
//            case 12:
//                if (!follower.isBusy()) {
//                    //grab pickup
//
//                    if (claw.getPosition() == CLOSE_CLAW) {
//                        follower.followPath(scorePickup4);
//                    }
//
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//                if (!follower.isBusy()) {
//                    //score pickup
//
//                    setPathState(14);  if there's time to park
//                    setPathState(-1);
//                }
//                break;

//            if there's time to park
//            case 14:
//                if (!follower.isBusy() && Math.abs(vSlideTargetPos - vSlidePos) < 30) {
//                    follower.followPath(park);
//                    setPathState(-1);
//                }
//                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        pathState = 0;

        poseUpdater = new PoseUpdater(hardwareMap);

        poseUpdater.setStartingPose(new Pose(10, 60, Math.toRadians(0)));

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        lsUp = new MotorEx(hardwareMap, "lsup");
        lsUp.setInverted(true);
        lsUp.resetEncoder();
        lsUp.setRunMode(Motor.RunMode.PositionControl);
        lsUp.setPositionCoefficient(VERT_LIFT_POSITION_COEFFICIENT);
        lsUp.setPositionTolerance(VERT_LIFT_POSITION_TOLERANCE);

        rsUp = new MotorEx(hardwareMap, "rsup");
        rsUp.resetEncoder();
        rsUp.setRunMode(Motor.RunMode.PositionControl);
        rsUp.setPositionCoefficient(VERT_LIFT_POSITION_COEFFICIENT);
        rsUp.setPositionTolerance(VERT_LIFT_POSITION_TOLERANCE);

        leftSwivelServo = new SimpleServo(hardwareMap, "leftSwivelServo", 0, 360, AngleUnit.DEGREES);
        ServoImplEx swivelLImpl = hardwareMap.get(ServoImplEx.class, "leftSwivelServo");
        swivelLImpl.setPwmRange(new PwmControl.PwmRange(500, 2500));

        wrist = new SimpleServo(hardwareMap, "wrist", 0, 360, AngleUnit.DEGREES);

        claw = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);
        ServoImplEx clawImpl = hardwareMap.get(ServoImplEx.class, "claw");
        clawImpl.setPwmRange(new PwmControl.PwmRange(500, 2500));

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        poseUpdater.update();
        dashboardPoseTracker.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        telemetryA.addData("x", poseUpdater.getPose().getX());
        telemetryA.addData("y", poseUpdater.getPose().getY());
        telemetryA.addData("heading", poseUpdater.getPose().getHeading());
        telemetryA.addData("total heading", poseUpdater.getTotalHeading());
        telemetryA.update();
    }
}
