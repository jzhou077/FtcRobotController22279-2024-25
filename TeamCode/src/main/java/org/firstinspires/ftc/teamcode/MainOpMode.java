package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_COEFFICIENT;
import static org.firstinspires.ftc.teamcode.RobotConstants.LIFT_POSITION_TOLERANCE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.opmode.MecanumDrive;

@Config
@TeleOp(name="MainOpMode")
public class MainOpMode extends LinearOpMode {
    private MecanumDrive drive;
    private MotorEx lsOut, rsOut, lsUp, rsUp;
    private ServoEx servo1, leftServo, rightServo;
    private int horizontalPosition = 0;
    private int horizontalTargetPosition = 0;
    private int verticalPosition = 0;
    private int verticalTargetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        lsOut = new MotorEx(hardwareMap, "lsout");
        lsOut.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        lsOut.setInverted(true);
        lsOut.resetEncoder();
        lsOut.setRunMode(Motor.RunMode.PositionControl);
        lsOut.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lsOut.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        rsOut = new MotorEx(hardwareMap, "rsout");
        rsOut.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rsOut.resetEncoder();
        rsOut.setRunMode(Motor.RunMode.PositionControl);
        rsOut.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        rsOut.setPositionTolerance(LIFT_POSITION_TOLERANCE);

//        lsOut.setRunMode(MotorEx.RunMode.PositionControl);
//        lsOut.setPositionCoefficient(0.003);
//        lsOut.setPositionTolerance(35);

        lsUp = new MotorEx(hardwareMap, "lsup");
        lsUp.setInverted(true);
        lsUp.resetEncoder();
        lsUp.setRunMode(Motor.RunMode.PositionControl);
        lsUp.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lsUp.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        rsUp = new MotorEx(hardwareMap, "rsup");
        rsUp.resetEncoder();
        rsUp.setRunMode(Motor.RunMode.PositionControl);
        rsUp.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        rsUp.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        waitForStart();
        while (opModeIsActive()) {
            horizontalPosition = lsOut.getCurrentPosition();
            verticalPosition = lsUp.getCurrentPosition();

            telemetry.addData("Ticks: ", horizontalPosition);
            telemetry.addData("Target Position: ", horizontalTargetPosition);
            telemetry.update();

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.25,
                                -gamepad1.left_stick_x * 0.25
                        ),
                        -gamepad1.right_stick_x * 0.25
                ));
            } else {

                //--------------------------------------------------------------------------------------
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.75,
                                -gamepad1.left_stick_x * 0.75
                        ),
                        -gamepad1.right_stick_x * 0.75
                ));
                //--------------------------------------------------------------------------------------
            }

            if (gamepad2.a) {

            }

            if (gamepad1.y) {
                verticalTargetPosition = 5500;
            } else if (gamepad1.a) {
                verticalTargetPosition = 0;
            }

            if (gamepad1.x) {
                horizontalTargetPosition = 0;
            } else if (gamepad1.b) {
                horizontalTargetPosition = 3000;
            }

            if (gamepad1.left_trigger > 0) {
                horizontalTargetPosition -= 10;
            } else if (gamepad1.right_trigger > 0) {
                horizontalTargetPosition += 10;
            }

            if (horizontalTargetPosition < 0) {
                horizontalTargetPosition = 0;
            } else if (horizontalTargetPosition > 2050) {
                horizontalTargetPosition = 2000;
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

            if (gamepad1.dpad_up) {
                verticalTargetPosition += 50;
            } else if (gamepad1.dpad_down) {
                verticalTargetPosition -= 50;
            }

            if (verticalTargetPosition < 0) {
                verticalTargetPosition = 0;
            } else if (verticalTargetPosition > 5500) {
                verticalTargetPosition = 5400;
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
    }
}
