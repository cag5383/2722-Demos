package frc.robot.subsystems.Intake;

public final class IntakeConstants {

    // Intake motor current limit
    public static final int kIntakeMotorCurrentLimit = 40;

    // Encoder values
    public static final double kIntakePositionConversionFactor = 1;

    // Intake speeds
    public static final double kCoralIntakeSpeed = -0.30;
    public static final double kCoralOuttakeSpeed = -0.25;
    public static final double kAlgaeIntakeSpeed = 0.25;
    public static final double kAlgaeOuttakeSpeed = -0.45;

    // PID Variables
    public static final double kIntakeP = 0;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;
    public static final double kIntakeMinOutputPower = 0;
    public static final double kIntakeMaxOutputPower = 0;

    // Intake Coral Position Setpoint
    public static final double kIntakeCoralForwardSetpoint = 0;

    //CANIVORE configs?
    //CAG: I don't actually know what the deal with this is.
    public static final String kCANbus = "ChargeCAN";

    //Intake CANRange Configs
    public static final double kIntakeFOV = 6.57;
    public static final double kIntakeProximityHysteresis = 0;
    public static final double kIntakeProximityThreshold = .15;


    //Reef Detection CANRange Configs
    public static final double kReefMaxDistance = 0.425;
    public static final double kReefHysteresis = 0.02;
    public static final double kReefFOV = 6.75;
    public static final double kReefSignalStrength = 2500;



  }