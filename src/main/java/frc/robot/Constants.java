// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final class canIDs {
    // Swerve CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;

    // CAN IDs of Elevator Motors
    public static final int kElevatorCANID = 11; // Added CAN ID
    public static final int kFollowerElevatorCANID = 12; // Added CAN ID

    // CAN IDs of Motor
    public static final int kWristCANID = 13; // Added CAN ID

    // Intake motor can ID
    public static final int kIntakeMotorCANID = 15; // Added CAN ID

    // Intake CANRange CAN IDs
    public static final int kCANrangeID = 2;
    public static final int kAlignerID = 7;

    // CAN IDs of Tray Motor
    public static final int kTrayCANID = 14; // Added CAN ID

  }

  public static class OperatorConstants {
    // Logitech controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorPort = 1;

    // Joystick Operator Deadband
    public static final double kOperatorDeadband = 0.15;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }
}