// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ONE_KAYLA = 64.5; //inches

    public static final int MOTOR_DRIVE_LEFT_ONE = 1; // SparkMax, NEO
    public static final int MOTOR_DRIVE_LEFT_TWO = 2; // SparkMax, NEO
    public static final int MOTOR_DRIVE_RIGHT_ONE = 3; // SparkMax, NEO
    public static final int MOTOR_DRIVE_RIGHT_TWO = 4; // SparkMax, NEO

    public static final double LEFT_DRIVE_PID1_P = 0.0147; // Velocity
    public static final double LEFT_DRIVE_PID1_I = 0;
    public static final double LEFT_DRIVE_PID1_D = 0;
    public static final double LEFT_DRIVE_PID1_F = 0.0;

    public static final double RIGHT_DRIVE_PID1_P = 0.0147;
    public static final double RIGHT_DRIVE_PID1_I = 0;
    public static final double RIGHT_DRIVE_PID1_D = 0;
    public static final double RIGHT_DRIVE_PID1_F = 0;

    public static final double OUTPUT_RANGE_MIN = -1;
    public static final double OUTPUT_RANGE_MAX = 1;

    public static final double GEAR_RATIO_DRIVE = 1.0 / 11.0;

    public static final int CURRENT_LIMIT_DRIVE_LEFT = 40;
    public static final int CURRENT_LIMIT_DRIVE_RIGHT = 40;

    public static final int DRIVER_CONTROLLER = 1;

    public static final double S_VOLTS = 0.13995;
    public static final double V_VOLTS = 2.8897;
    public static final double A_VOLTS = 0.17688;

    public static final double S_VOLTS_LEFT = 0.14489;
    public static final double V_VOLTS_LEFT = 2.8997;
    public static final double A_VOLTS_LEFT = 0.13809;

    public static final double S_VOLTS_RIGHT = 0.13995;
    public static final double V_VOLTS_RIGHT = 2.8897;
    public static final double A_VOLTS_RIGHT = 0.17688;

    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    public static final double TRACK_WIDTH_METERS = 0.75642; // experimentally determined

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    public static final double MAX_SPEED_METERS_PER_SECOND = 2;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

    public static final double DRIVE_WHEEL_DIAMETER = Units.inchesToMeters(5.775); // experimentally determined
    public static final boolean GYRO_REVERSED = true;
    public static final int SECONDS_PER_MINUTE = 60;

    public static final double OPEN_LOOP_RAMP_RATE = 0.05;
}
