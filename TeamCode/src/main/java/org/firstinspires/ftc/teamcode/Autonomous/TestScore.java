package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_SCORE_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_SCORE_UP_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.FULL_VERT_EXTENSION;
import static org.firstinspires.ftc.teamcode.RobotConstants.HIGH_CHAMBER_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.OPEN_CLAW;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="Test Score")
public class TestScore extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private final Pose startPose = new Pose(9, 72, Math.toRadians(180));
    private final Pose scorePose = new Pose(34, 72, Math.toRadians(180));
    private MotorEx lsUp, rsUp;
    private ServoEx leftSwivelServo, claw;
    private int vSlideTargetPos, vSlidePos;

    private PathChain scorePreload;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                pathState = -1;
        }
    }

    public void scoreSpecimen() {
        //bring slides to scoring height
        //move slides down about 1.5 inch
        //open claw
        leftSwivelServo.setPosition(ARM_SCORE_DOWN_POSITION);
        if (Math.abs(leftSwivelServo.getPosition() - ARM_SCORE_DOWN_POSITION) < 0.025) {
            claw.setPosition(OPEN_CLAW);
        }
    }

    public void updateVSlides() {
        vSlidePos = lsUp.getCurrentPosition();

        if (vSlideTargetPos < 0) {
            vSlideTargetPos = 0;
        } else if (vSlideTargetPos > FULL_VERT_EXTENSION) {
            vSlideTargetPos = 5400;
        }

        lsUp.setTargetPosition(vSlideTargetPos);
        rsUp.setTargetPosition(vSlideTargetPos);

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

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
    }
}
