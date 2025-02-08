package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="Red Specimen Auto", group="Autonomous OpModes")
public class RedSpecimenAuto extends OpMode {

    private ServoEx arm, wrist, claw;
    private MotorEx lsUp, rsUp;
    private int vHeight, vTargetHeight, hHeight, hTargetHeight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(135, 72, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(110, 72, Math.toRadians(0)); //1
    private final Pose push1Pose = new Pose(96, 108, Math.toRadians(0)); // 2
    private final Pose push1ControlPose = new Pose(126, 106);
    private final Pose push2Pose = new Pose(84, 118, Math.toRadians(0)); //3
    private final Pose push2ControlPose = new Pose(67, 109);
    private final Pose push3Pose = new Pose(134, 118, Math.toRadians(0)); //4
    private final Pose push4Pose = new Pose(84, 130, Math.toRadians(0)); //5
    private final Pose push4ControlPose = new Pose(79, 108);
    private final Pose push5Pose = new Pose(134, 130, Math.toRadians(0)); //6
    private final Pose prepGrabPose = new Pose(131, 130, Math.toRadians(180)); // 7
    private final Pose reloadPose = new Pose(131, 114, Math.toRadians(180)); // 8
    private final Pose scorePickup1Pose = new Pose(110, 75, Math.toRadians(0)); //9
    private final Pose scorePickup2Pose = new Pose(110, 78, Math.toRadians(0)); // 11
    private final Pose scorePickup3Pose = new Pose(110, 81, Math.toRadians(0)); // 13

    private PathChain scorePreload, pushSamples, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
    private boolean actionBusy = false;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        pushSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(push1ControlPose), new Point(push1Pose)))
                .setConstantHeadingInterpolation(push1Pose.getHeading())
                .addPath(new BezierCurve(new Point(push1Pose), new Point(push2ControlPose), new Point(push2Pose)))
                .setConstantHeadingInterpolation(push2Pose.getHeading())
                .addPath(new BezierLine(new Point(push2Pose), new Point(push3Pose)))
                .setConstantHeadingInterpolation(push3Pose.getHeading())
                .addPath(new BezierCurve(new Point(push3Pose), new Point(push4ControlPose), new Point(push4Pose)))
                .setConstantHeadingInterpolation(push4Pose.getHeading())
                .addPath(new BezierLine(new Point(push4Pose), new Point(push5Pose)))
                .setConstantHeadingInterpolation(push5Pose.getHeading())
                .addPath(new BezierLine(new Point(push5Pose), new Point(prepGrabPose)))
                .setLinearHeadingInterpolation(push5Pose.getHeading(), prepGrabPose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prepGrabPose), new Point(reloadPose)))
                .setConstantHeadingInterpolation(reloadPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reloadPose), new Point(scorePickup1Pose)))
                .setLinearHeadingInterpolation(reloadPose.getHeading(), scorePickup1Pose.getHeading(), 1.5)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePickup1Pose), new Point(reloadPose)))
                .setLinearHeadingInterpolation(scorePickup1Pose.getHeading(), reloadPose.getHeading(), 1.5)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reloadPose), new Point(scorePickup2Pose)))
                .setLinearHeadingInterpolation(reloadPose.getHeading(), scorePickup2Pose.getHeading(), 1.5)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePickup2Pose), new Point(reloadPose)))
                .setLinearHeadingInterpolation(scorePickup2Pose.getHeading(), reloadPose.getHeading(), 1.5)
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(reloadPose), new Point(scorePickup3Pose)))
                .setLinearHeadingInterpolation(reloadPose.getHeading(), scorePickup3Pose.getHeading(), 1.5)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePickup3Pose), new Point(reloadPose)))
                .setLinearHeadingInterpolation(scorePickup3Pose.getHeading(), reloadPose.getHeading(), 1.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                arm.setPosition(ARM_SCORE_DOWN_POSITION);
                wrist.setPosition(WRIST_UPSIDE_DOWN);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (follower.getPose().getX() >= scorePreloadPose.getX() - 0.5) {
                    if (!actionBusy) {
                        actionBusy = true;
                        actionTimer.resetTimer();
                    } else if (actionTimer.getElapsedTimeSeconds() > 2) {
                        vTargetHeight = HIGH_CHAMBER_POS;
                        if (actionTimer.getElapsedTimeSeconds() > 4) {
                            claw.setPosition(OPEN_CLAW);
                            if (actionTimer.getElapsedTimeSeconds() > 5) {
                                //                        follower.followPath(pushSamples);
                                actionBusy = false;
                                follower.followPath(park);
                                vTargetHeight = 0;
                                setPathState(2);
                            }
                        }
                    }
                }
                break;
//            case 2:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    vTargetHeight = GRAB_SPECIMEN_HEIGHT;
//                    arm.setPosition(ARM_GRAB_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_UP);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup1);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    if (!actionBusy) {
//                        actionBusy = true;
//                        actionTimer.resetTimer();
//                    } else if (actionTimer.getElapsedTimeSeconds() > 2) {
//                        claw.setPosition(CLOSE_CLAW);
//                        actionBusy = false;
//                        setPathState(5);
//                    }
//                }
//                break;
//            case 5:
//                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    follower.followPath(scorePickup1);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    arm.setPosition(ARM_SCORE_DOWN_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_DOWN);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (follower.getPose().getX() >= scorePreloadPose.getX() - 0.5) {
//                    if (!actionBusy) {
//                        actionBusy = true;
//                        actionTimer.resetTimer();
//                    }
//                    vTargetHeight = HIGH_CHAMBER_POS;
//                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                        claw.setPosition(OPEN_CLAW);
//                        if (actionTimer.getElapsedTimeSeconds() > 1.75) {
//                            follower.followPath(grabPickup2);
//                            actionBusy = false;
//                            setPathState(8);
//                        }
//                    }
//                }
//                break;
//            case 8:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    vTargetHeight = GRAB_SPECIMEN_HEIGHT;
//                    arm.setPosition(ARM_GRAB_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_UP);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy()) {
//                    if (!actionBusy) {
//                        actionBusy = true;
//                        actionTimer.resetTimer();
//                    } else if (actionTimer.getElapsedTimeSeconds() > 2) {
//                        claw.setPosition(CLOSE_CLAW);
//                        actionBusy = false;
//                        setPathState(10);
//                    }
//                }
//                break;
//            case 10:
//                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    follower.followPath(scorePickup2);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    arm.setPosition(ARM_SCORE_DOWN_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_DOWN);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if (follower.getPose().getX() >= scorePreloadPose.getX() - 0.5) {
//                    if (!actionBusy) {
//                        actionBusy = true;
//                        actionTimer.resetTimer();
//                    }
//                    vTargetHeight = HIGH_CHAMBER_POS;
//                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                        claw.setPosition(OPEN_CLAW);
//                        if (actionTimer.getElapsedTimeSeconds() > 1.75) {
//                            follower.followPath(grabPickup3);
//                            actionBusy = false;
//                            setPathState(13);
//                        }
//                    }
//                }
//                break;
//            case 13:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    vTargetHeight = GRAB_SPECIMEN_HEIGHT;
//                    arm.setPosition(ARM_GRAB_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_UP);
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if (!follower.isBusy()) {
//                    if (!actionBusy) {
//                        actionBusy = true;
//                        actionTimer.resetTimer();
//                    } else if (actionTimer.getElapsedTimeSeconds() > 2) {
//                        claw.setPosition(CLOSE_CLAW);
//                        actionBusy = false;
//                        setPathState(15);
//                    }
//                }
//                break;
//            case 15:
//                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    follower.followPath(scorePickup3);
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    arm.setPosition(ARM_SCORE_DOWN_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_DOWN);
//                    setPathState(17);
//                }
//                break;
//            case 17:
//                if (follower.getPose().getX() >= scorePreloadPose.getX() - 0.5) {
//                    if (!actionBusy) {
//                        actionBusy = true;
//                        actionTimer.resetTimer();
//                    }
//                    vTargetHeight = HIGH_CHAMBER_POS;
//                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                        claw.setPosition(OPEN_CLAW);
//                        if (actionTimer.getElapsedTimeSeconds() > 1.75) {
//                            follower.followPath(park);
//                            setPathState(18);
//                        }
//                    }
//                }
//                break;
//            case 18:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    vTargetHeight = GRAB_SPECIMEN_HEIGHT;
//                    arm.setPosition(ARM_GRAB_POSITION);
//                    wrist.setPosition(WRIST_UPSIDE_UP);
//                    setPathState(-1);
//                }
//                break;
        }
    }

    @Override
    public void init() {

        arm = new SimpleServo(hardwareMap, "leftSwivelServo", 0, 360);
        wrist = new SimpleServo(hardwareMap, "wrist", 0, 360);
        claw = new SimpleServo(hardwareMap, "claw", 0, 360);

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

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        wrist.setPosition(WRIST_UPSIDE_UP);
        claw.setPosition(CLOSE_CLAW);
    }

    @Override
    public void loop() {

        updateSlides();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void setPathState(int pstate) {
        pathState = pstate;
        pathTimer.resetTimer();
    }

    public void updateSlides() {
        vHeight = lsUp.getCurrentPosition();

        lsUp.setTargetPosition(vTargetHeight);
        rsUp.setTargetPosition(vTargetHeight);

        if (lsUp.atTargetPosition()) {
            lsUp.set(0);
        } else {
            lsUp.set(1);
        }

        if (rsUp.atTargetPosition()) {
            rsUp.set(0);
        } else {
            rsUp.set(1);
        }
    }

}
