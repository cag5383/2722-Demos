package frc.robot.subsystems.Tray;

public  final class TrayConstants {

    // Current Limit of Motor
    public static final int kSmartCurrentLimit = 40;

    // Encoder Values
    public static final double kPositionConversionFactor = 360;
    public static final double kVelocityConversionFactor = 1;
    public static final double kZeroOffset = 0.0200;

    // PID Variables
    public static final double kTrayP = 0.01;
    public static final double kTrayI = 0;
    public static final double kTrayD = 0;
    public static final double kTrayMinOutputPower = -0.1;
    public static final double kTrayMaxOutputPower = 0.1;

    // Soft limit values
    public static final double kTrayForwardSoftLimit = 47;
    public static final double kTrayReverseSoftLimit = 5;

    // Setpoints
    public static final double kIntakeTraySetpoint = 46;
    public static final double kTrayHangPositionSetpoint = 6;

    // Tray speeds
    public static final double kTrayUpSpeed = 0;
    public static final double kTrayDownSpeed = 0;
  }