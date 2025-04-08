package frc.robot.subsystems.Tray;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Tray extends SubsystemBase {
  /** Creates a new traySubsystem. */

  // Declares motor and motor config
  private SparkFlex m_trayMotor;

  private SparkFlexConfig trayMotorConfig;

  // Declare encoder
  private AbsoluteEncoder trayEncoder; // May need to be absolute

  // Declares PID variables
  // CAG: PID was probably overkill here once we gave up on the hang. but if we're comfortable with it, it
  // certainly gets the job done.
  // PID is really for things that need to be held at relatively precise targets
  // even when there are disturbances.
  // That's not really the case for something we're just trying to flip up and
  // then down.
  private SparkClosedLoopController tray_pid;

  // Declare soft limits
  private SoftLimitConfig traySoftLimitConfig;

  public Tray() {

    configureTrayMotor();
    trayEncoder = m_trayMotor.getAbsoluteEncoder();
    tray_pid = m_trayMotor.getClosedLoopController();

  }

  public void setTrayMotorSpeed(double inputSpeed) {
    // Sets the speed for the tray to move freely
    m_trayMotor.set(inputSpeed);
  }

  public void stopTrayMotor() {
    // Stops the tray
    m_trayMotor.stopMotor();
    ;
  }

  public void setTrayPosition(double position) {
    // Moves the tray to the desired position using PID and setpoint position
    tray_pid.setReference(position, SparkFlex.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Tray Current", m_trayMotor.getOutputCurrent());
    SmartDashboard.putNumber("Tray Position: ", trayEncoder.getPosition());
  }

  private void configureTrayMotor() {
    // CAG: Same deal as in other subsystems - I think we can consolidate
    // motor configs into a method to make this more readable.

    // Initializes motor and motor config
    m_trayMotor = new SparkFlex(canIDs.kTrayCANID, MotorType.kBrushless);
    trayMotorConfig = new SparkFlexConfig();

    // Sets configs
    trayMotorConfig.smartCurrentLimit(TrayConstants.kSmartCurrentLimit);
    trayMotorConfig.idleMode(IdleMode.kBrake);

    // Set soft limits
    traySoftLimitConfig = new SoftLimitConfig();
    traySoftLimitConfig.forwardSoftLimit(TrayConstants.kTrayForwardSoftLimit);
    traySoftLimitConfig.reverseSoftLimit(TrayConstants.kTrayReverseSoftLimit);
    traySoftLimitConfig.forwardSoftLimitEnabled(true);
    traySoftLimitConfig.reverseSoftLimitEnabled(true);

    trayMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    trayMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    trayMotorConfig.apply(traySoftLimitConfig);

    trayMotorConfig.absoluteEncoder.zeroOffset(TrayConstants.kZeroOffset);
    trayMotorConfig.absoluteEncoder.positionConversionFactor(TrayConstants.kPositionConversionFactor); // Test these
                                                                                                       // values
    trayMotorConfig.absoluteEncoder.velocityConversionFactor(TrayConstants.kVelocityConversionFactor);

    // Sets up the Closed Loop Controller
    trayMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    trayMotorConfig.closedLoop.p(TrayConstants.kTrayP);
    trayMotorConfig.closedLoop.i(TrayConstants.kTrayI);
    trayMotorConfig.closedLoop.d(TrayConstants.kTrayD);
    trayMotorConfig.closedLoop.outputRange(TrayConstants.kTrayMinOutputPower, TrayConstants.kTrayMaxOutputPower);

    // Configures motor to desired configs
    m_trayMotor.configure(trayMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
}