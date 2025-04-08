package frc.robot.subsystems.Elevator;

public class ElevatorConstants {

    // Current Limit of Motors
    public static final int kSmartCurrentLimit = 40;

    // Encoder Conversion Factors
    public static final double kPositionConversionFactor = 100;
    public static final double kVelocityConversionFactor = 1;

    // String Pot
    public static final int kStringPotPort = 0;
    public static final double kElevatorStringPotMaxRange = 50; // Inches
    public static final double kElevatorStringPotOffset = -0.44; // Inches

    // String Pot Conversion Factors
    // 5747.16 * 25/12
    public static final double kMaxEncoderDegreesReading = 12085;
    public static final double kMaxStringPotInchesReading = 47.307;
    public static final double kDegreesPerInch = 253.0968;
    public static final double kInchesPerDegree = 0.003951;

    // PID Variables
    public static final double kElevatorP = 0.9;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorMinOutputPower = -0.80;
    public static final double kElevatorMaxOutputPower = 0.95;
    public static final double kElevatorPositionTolerance = .05; // Test this value

    // Soft limit values
    // 5700 * 25/12 (25/12 is 2.0833)
    public static final double kElevatorForwardSoftLimit = 4.865;
    public static final double kElevatorReverseSoftLimit = .0475;

    // Elevator Level Setpoints in elevator encoder degrees
    public static final double kElevatorL1Setpoint = 0;
    public static final double kElevatorL2Setpoint = 0.98;
    public static final double kElevatorL3Setpoint = 2.50; // 2.45 old value
    public static final double kElevatorL4Setpoint = 4.83;
    public static final double kElevatorIntakeSetpoint = 0.04756;
    public static final double kElevatorHangPositionSetpoint = 0;
    public static final double kElevatorUpperAlgaeRemove = 3.25;
    public static final double kElevatorLowerAlgaeRemove = 1.75;
    public static final double kElevatorProcessorSetpoint = 0.9;
    public static final double kElevatorRestSetpoint = 0.04756;

    // Elevator Speed
    public static final double kElevatorManualSpeed = 0.80;
  }