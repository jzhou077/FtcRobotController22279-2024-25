package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@TeleOp(name="MainOpMode")
public class MainOpMode extends LinearOpMode {
    private Follower follower; //**
    private MotorEx lsOut, rsOut, lsUp, rsUp;
    private ServoEx leftSwivelServo, rightSwivelServo, wrist, claw, intakeSwivel;
    private CRServo intakeR, intakeL;
    private int horizontalPosition = 0;
    private int horizontalTargetPosition = 0;
    private int verticalPosition = 0;
    private int verticalTargetPosition = 0;
    private int macro1_step, macro2_step, macro3_step, macro4_step, macro5_step, macro6_step, macro7_step, macro8_step;
    private Timer macro8_timer, macro7_timer;
    private int reversed = 1;
    private GamepadEx gp1, gp2;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeBot();

        waitForStart();
        while (opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * 0.25 * reversed, -gamepad1.left_stick_x * 0.25 * reversed, -gamepad1.right_stick_x * 0.25 * reversed, true);
                follower.update();
            } else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * 0.75 * reversed, -gamepad1.left_stick_x * 0.75 * reversed, -gamepad1.right_stick_x * 0.75 * reversed, true);
                follower.update();
            } else {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * 0.5 * reversed, -gamepad1.left_stick_x * 0.5 * reversed, -gamepad1.right_stick_x * 0.5 * reversed, true);
                follower.update();
            }

            if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER) && gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                reversed = -reversed;
            }
            //---------------------------------------------------------------------------------------

            updateVSlides();
            updateHSlides();
            updateTelemetry();

            if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                intakeOff();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                intakeReverse();
            }

            if (gamepad2.y) {
                positionWrist();
            }

//            if (gamepad2.left_bumper && isClawClosed) {
//                alignHighBasket();
//            } else if (gamepad2.right_bumper) {
//                resetFromHighBasket();
//            }

            extend_intake();
            retract_intake();
            specimen_macro();
            hang();

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                closeClaw();
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                openClaw();
            }
        }
    }

    //all functions start here

    public void initializeBot() {
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0)); //do i really need this

        lsOut = new MotorEx(hardwareMap, "lsout");
        lsOut.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        lsOut.setInverted(true);
        lsOut.resetEncoder();
        lsOut.setRunMode(Motor.RunMode.PositionControl);
        lsOut.setPositionCoefficient(HOR_LIFT_POSITION_COEFFICIENT);
        lsOut.setPositionTolerance(HOR_LIFT_POSITION_TOLERANCE);

        rsOut = new MotorEx(hardwareMap, "rsout");
        rsOut.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rsOut.resetEncoder();
        rsOut.setRunMode(Motor.RunMode.PositionControl);
        rsOut.setPositionCoefficient(HOR_LIFT_POSITION_COEFFICIENT);
        rsOut.setPositionTolerance(HOR_LIFT_POSITION_TOLERANCE);

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
        ServoImplEx wristImpl = hardwareMap.get(ServoImplEx.class, "wrist");
        wristImpl.setPwmRange(new PwmControl.PwmRange(500, 2500));

        claw = new SimpleServo(hardwareMap, "claw", 0, 360, AngleUnit.DEGREES);
        ServoImplEx clawImpl = hardwareMap.get(ServoImplEx.class, "claw");
        clawImpl.setPwmRange(new PwmControl.PwmRange(500, 2500));

        intakeL = new CRServo(hardwareMap, "leftIntake");
        intakeR = new CRServo(hardwareMap, "rightIntake");

        intakeSwivel = new SimpleServo(hardwareMap, "intakeSwivel", 0, 360, AngleUnit.DEGREES);
        ServoImplEx intakeSwivelImpl = hardwareMap.get(ServoImplEx.class, "claw");
        intakeSwivelImpl.setPwmRange(new PwmControl.PwmRange(500, 2500));

        macro7_timer = new Timer();
        macro8_timer = new Timer();

        macro1_step = macro2_step = macro3_step = macro4_step = macro5_step = macro6_step = macro7_step = macro8_step = 0;

        follower.startTeleopDrive();
    }

    public void positionServo() {
        leftSwivelServo.setPosition(ARM_SCORE_UP_POSITION); //0.5
    }

    public void positionWrist() {
        wrist.setPosition(WRIST_UPSIDE_UP); //200 is good value, 0.875 for center down, 0.2 for center up
    }

    public void openClaw() {
        claw.setPosition(OPEN_CLAW);
    }

    public void closeClaw() {
        claw.setPosition(CLOSE_CLAW);
    }

    public void updateVSlides() {
        verticalPosition = lsUp.getCurrentPosition();

        if (verticalTargetPosition < 0) {
            verticalTargetPosition = 0;
        } else if (verticalTargetPosition > FULL_VERT_EXTENSION) {
            verticalTargetPosition = FULL_VERT_EXTENSION;
        }

        lsUp.setTargetPosition(verticalTargetPosition);
        rsUp.setTargetPosition(verticalTargetPosition);

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

    public void updateHSlides() {
        horizontalPosition = lsOut.getCurrentPosition();

        if (verticalPosition > 800) {
            horizontalTargetPosition = 0;
        }

        if (horizontalTargetPosition < 0) {
            horizontalTargetPosition = 0;
        } else if (horizontalTargetPosition > FULL_HOR_EXTENSION) {
            horizontalTargetPosition = FULL_HOR_EXTENSION;
        }

        lsOut.setTargetPosition(horizontalTargetPosition);
        rsOut.setTargetPosition(horizontalTargetPosition);

        if (rsOut.atTargetPosition()) {
            rsOut.set(0);
        } else {
            rsOut.set(1);
        }

        if (lsOut.atTargetPosition()) {
            lsOut.set(0);
        } else {
            lsOut.set(1);
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Horizontal Position: ", horizontalPosition);
        telemetry.addData("H Target Position: ", horizontalTargetPosition);

        telemetry.addData("Vertical Position: ", verticalPosition);

        telemetry.addData("Intake Swivel Pos:", intakeSwivel.getPosition());
        telemetry.addData("Extension macro step: ", macro7_step);
        telemetry.addData("Retraction macro step: ", macro8_step);
        telemetry.update();
    }

    public void alignHighBasket() {
        switch(macro1_step) {
            case 0:
                if (claw.getPosition() == CLOSE_CLAW) {
                    leftSwivelServo.setPosition(ARM_SCORE_UP_POSITION);
                    wrist.setPosition(WRIST_UPSIDE_DOWN);
                    verticalTargetPosition = FULL_VERT_EXTENSION;
                    macro1_step = 1;
                }
                break;
            case 1:
                if (wrist.getPosition() == WRIST_UPSIDE_DOWN && Math.abs(verticalPosition - verticalTargetPosition) < 50) {
                    leftSwivelServo.setPosition(ARM_SCORE_DOWN_POSITION);
                    macro1_step = 0;
                }
                break;
            default:
                macro1_step = 0;
        }
    }

    public void resetFromHighBasket() {
        switch (macro2_step) {
            case 0:
                openClaw();
                macro2_step = 1;
                break;
            case 1:
                if (claw.getPosition() == OPEN_CLAW) {
                    leftSwivelServo.setPosition(ARM_SCORE_UP_POSITION);
                    verticalTargetPosition = 0;
                    macro2_step = 2;
                }
                break;
            case 2:
                if (Math.abs(verticalPosition - verticalTargetPosition) < 30 && leftSwivelServo.getPosition() == ARM_SCORE_UP_POSITION) {
                    leftSwivelServo.setPosition(ARM_SCORE_DOWN_POSITION);
                    wrist.setPosition(WRIST_UPSIDE_UP);
                    macro2_step = 0;
                }
                break;
            default:
                macro2_step = 0;
        }
    }

    public void hang() {
        switch (macro3_step) {
            case 0:
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    verticalTargetPosition = HANG_HEIGHT;
                    macro3_step = 1;
                }
                break;
            case 1:
                if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    verticalTargetPosition = 0;
                    macro3_step = 0;
                }
                break;
            default:
                macro3_step = 0;
        }
    }

    public void prepSpecimenScore() {
        leftSwivelServo.setPosition(ARM_SCORE_DOWN_POSITION);
//        leftSwivelServo.setPosition(ARM_SCORE_UP_POSITION);
        wrist.setPosition(WRIST_UPSIDE_DOWN);
    }

    public void scoreSpecimen() {
//            verticalTargetPosition = HIGH_CHAMBER_POS;
            verticalTargetPosition = HIGH_CHAMBER_POS2;
//            if (Math.abs(verticalTargetPosition - verticalPosition) <= 200) {
//            openClaw();
//          }
    }

    public void prepGrabSpecimen() {
        verticalTargetPosition = GRAB_SPECIMEN_HEIGHT;
        wrist.setPosition(WRIST_UPSIDE_UP);
        leftSwivelServo.setPosition(ARM_GRAB_POSITION);
        openClaw();
    }

    public void specimen_macro() {
        switch (macro4_step) {
            case 0:
                if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                    macro4_step = 1;
                }
                break;
            case 1:
                prepGrabSpecimen();
                if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                    macro4_step = 2;
                }
                break;
            case 2:
                prepSpecimenScore();
                if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                    scoreSpecimen();
                    macro4_step = 3;
                } else if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                    macro4_step = 1;
                }
            case 3:
                scoreSpecimen();
                if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                    macro4_step = 1;
                } else if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                    macro4_step = 2;
                }
            default:
                macro4_step = 0;
        }
    }

    public void extend_intake() {
        switch(macro7_step) {
            case 0:
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    macro8_step = 0;
                    macro7_timer.resetTimer();
                    intakeSwivel.setPosition(INTAKE_SWIVEL_UP);
                    macro7_step = 1;
                }
                break;
            case 1:
                if (macro7_timer.getElapsedTimeSeconds() > 1) {
                    horizontalTargetPosition = FULL_HOR_EXTENSION;
                    macro7_step = 2;
                }
                break;
            case 2:
                if (Math.abs(horizontalPosition - horizontalTargetPosition) <= 50) {
                    intakeSwivel.setPosition(INTAKE_SWIVEL_DOWN);
                    intakeForward();
                    macro7_step = 0;
                }
                break;
            default:
                macro7_step = 0;
        }
    }

    public void retract_intake() {
        switch (macro8_step) {
            case 0:
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    macro7_step = 0;
                    intakeOff();
                    macro8_timer.resetTimer();
                    intakeSwivel.setPosition(INTAKE_SWIVEL_UP);
                    macro8_step = 1;
                    }
                break;
            case 1:
                if (macro8_timer.getElapsedTimeSeconds() > 1) {
                    horizontalTargetPosition = 0;
                    macro8_step = 2;
                }
                break;
            case 2:
                if (Math.abs(horizontalPosition - horizontalTargetPosition) <= 50) {
                    macro8_timer.resetTimer();
                    intakeSwivel.setPosition(INTAKE_SWIVEL_DOWN);
                    macro8_step = 0;
                }
                break;
            default:
                macro8_step = 0;
        }
    }

    public void intakeForward() {
        intakeL.set(1);
        intakeR.set(-1);
    }

    public void intakeOff() {
        intakeL.set(0);
        intakeR.set(0);
    }

    public void intakeReverse() {
        intakeL.set(-1);
        intakeR.set(1);
    }
}
