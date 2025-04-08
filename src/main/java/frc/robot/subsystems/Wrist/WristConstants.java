package frc.robot.subsystems.Wrist;

public  class WristConstants {

    // Current Limit of Motor
    public static final int kSmartCurrentLimit = 40;

    // Encoder Values
    public static final double kPositionConversionFactor = 360;
    public static final double kVelocityConversionFactor = 1;
    public static final double kZeroOffset = 0.2438;

    // PID Variables
    public static final double kWristP = 0.01; // test .005
    public static final double kWristI = 0;
    public static final double kWristD = 0;
    public static final double kWristMinOutputPower = -0.2;
    public static final double kWristMaxOutputPower = 0.2;
    public static final double kWristPositionTolerance = 2.7; // Test this value

    // Soft limit values
    public static final double kWristForwardSoftLimit = 222;
    public static final double kWristReverseSoftLimit = 15;

    // Wrist Level Setpoints
    public static final double kWristL1Setpoint = 0;
    public static final double kWristL2Setpoint = 188;
    public static final double kWristL3Setpoint = 188;
    public static final double kWristL4Setpoint = 165;
    public static final double kWristIntakeSetpoint = 220;
    public static final double kWristAlgaeIntakeSetpoint = 42;
    public static final double kWristAlgaeOuttakeSetpoint = 42;
    public static final double kWristHangPositionSetpoint = 0;
    public static final double kWristProcessorSetpoint = 17;
    public static final double kWristTrayStowSetpoint = 188;

    // Value may be 150 degrees
    public static final double kWristElevatorSafeSetpoint = 188; // Used to make sure the wrist doesn't hit the elevator
                                                                 // while the elevator moving

    // Wrist Speed
    public static final double kWristManualSpeed = 0.2;
  }