package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.MecanumDrive;

@Config
@TeleOp(name="MainOpMode")
public class MainOpMode extends LinearOpMode {
    private MecanumDrive drive;
    private MotorEx lsOut;
    private MotorEx lsUp;
    private MotorEx rsUp;
    private int outTargetPosition = 0;
    private int lsOutPosition = 0;
    private int verticalPosition = 0;
    private int verticalTargetPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        lsOut = new MotorEx(hardwareMap, "lsout");
        lsOut.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        lsOut.setInverted(true);
        lsOut.resetEncoder();

//        lsOut.setRunMode(MotorEx.RunMode.PositionControl);
//        lsOut.setPositionCoefficient(0.003);
//        lsOut.setPositionTolerance(35);

        lsUp = new MotorEx(hardwareMap, "lsup");
        lsUp.setInverted(true);
        lsUp.resetEncoder();
        lsUp.setRunMode(Motor.RunMode.PositionControl);
        lsUp.setPositionCoefficient(0.003);
        lsUp.setPositionTolerance(35);

        rsUp = new MotorEx(hardwareMap, "rsup");
        rsUp.resetEncoder();
        rsUp.setRunMode(Motor.RunMode.PositionControl);
        rsUp.setPositionCoefficient(0.003);
        rsUp.setPositionTolerance(35);

        waitForStart();
        while (opModeIsActive()) {
            lsOutPosition = lsOut.getCurrentPosition();
            verticalPosition = lsUp.getCurrentPosition();

            telemetry.addData("Ticks: ", verticalPosition);
            telemetry.addData("Target Position: ", verticalTargetPosition);
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

            if (gamepad1.left_trigger > 0 && lsOutPosition > 0) {
                lsOut.set(-0.3);
            } else if (gamepad1.right_trigger > 0 && lsOutPosition < 2900) {
                lsOut.set(0.3);
            } else {
                lsOut.set(0);
            }

//            if (outTargetPosition < 0) {
//                outTargetPosition = 0;
//            } else if (outTargetPosition > 2900) {
//                outTargetPosition = 2800;
//            }

//            lsOut.setTargetPosition(outTargetPosition);

//            if (lsOut.atTargetPosition()) {
//                lsOut.set(0);
//            } else {
//                lsOut.set(1);
//            }

            if (gamepad1.dpad_up) {
                verticalTargetPosition += 20;
            } else if (gamepad1.dpad_down) {
                verticalTargetPosition -= 20;
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
