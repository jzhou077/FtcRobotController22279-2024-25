package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static final int ENCODER_TICKS_PER_INCH = (int) (5348 / 33.1);
    public static double VERT_LIFT_POSITION_COEFFICIENT = 0.001;
    public static double HOR_LIFT_POSITION_COEFFICIENT = 0.005;
    public static int VERT_LIFT_POSITION_TOLERANCE = 25;
    public static int HOR_LIFT_POSITION_TOLERANCE = 30;
    public static int FULL_VERT_EXTENSION = 5500;
    public static int FULL_HOR_EXTENSION = 2050;
    public static int HIGH_CHAMBER_POS = ENCODER_TICKS_PER_INCH * 12;
    public static double height_tester = 6.5;
    public static int HIGH_CHAMBER_POS2 = (int) (ENCODER_TICKS_PER_INCH * height_tester);
    public static int GRAB_SPECIMEN_HEIGHT = 0;
    public static int HANG_HEIGHT = 5000;
    public static double ARM_SCORE_UP_POSITION = 0.16;
    public static double ARM_SCORE_DOWN_POSITION = 0.07;
    public static double ARM_GRAB_POSITION = 0;
    public static double OPEN_CLAW = 1;
    public static double CLOSE_CLAW = 0.8;
    public static double WRIST_UPSIDE_UP = 0.62;
    public static double WRIST_UPSIDE_DOWN = 0;
    public static double STRAFE_X;
    public static double INTAKE_SWIVEL_UP = 0.25;
    public static double INTAKE_SWIVEL_DOWN = 0.59;
}
