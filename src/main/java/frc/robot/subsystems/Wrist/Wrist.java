// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.StateTracker.StateTracker;
import frc.robot.subsystems.StateTracker.WristData;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Wrist extends SubsystemBase {
  /** Creates a new WristSubsystem. */

  // Declares motor and motor config
  private SparkFlex m_wristMotor;

  // Declares encoder
  private AbsoluteEncoder wristEncoder; // May need to be absolute

  // Declares PID variables
  private SparkClosedLoopController wrist_pid;

  // Declare soft limits
  private SoftLimitConfig wristSoftLimitConfig;

  //Data structure periodically returned to the StateTracker.
  private WristData wristData;
  private StateTracker m_stateTracker;

  public Wrist(StateTracker tracker) {
    configureWristMotor();
    m_stateTracker = tracker;

    // Gets PID controller from motor
    wrist_pid = m_wristMotor.getClosedLoopController();
  }

  public void setWristMotorSpeed(double inputSpeed) {
    // Sets the speed for the wrist to move freely

    // CAG: I'm not sure whether this method is necessary.
    // Seems kind of redundant with runWristManually, same as in Elevator.
    m_wristMotor.set(inputSpeed);
  }

  public void runWristManually(double yWristSpeed) {
    // Controls the wrist manually with a joystick
    if (yWristSpeed >= .5)
      yWristSpeed = WristConstants.kWristManualSpeed;
    if (yWristSpeed <= -.5)
      yWristSpeed = -WristConstants.kWristManualSpeed;

    m_wristMotor.set(yWristSpeed);
  }

  public void stopWristMotor() {
    // Stops the wrist
    m_wristMotor.stopMotor();
  }

  public void setWristPosition(double position) {
    // Moves the wrist to the desired position using PID and setpoint position
    wrist_pid.setReference(position, SparkFlex.ControlType.kPosition);
  }

  public double getWristEncoderPosition() {
    // Gets the position of the wrist based on the encoder
    return wristEncoder.getPosition();
  }

  public boolean atWristSetpoint(double inputWristPosition) {
    // Returns if wrist is at setpoint

    // CAG: Having to pass in the position I'm looking for is probably bad practice.
    // It means that any commands/subsystems that want to ask whether the wrist
    // is where it's supposed to be also have to know what to ask about.
    //You have to pass around and keep track of more information than is really required.
    return Math.abs(inputWristPosition - getWristEncoderPosition()) < WristConstants.kWristPositionTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    publishWristData();

    // Puts values on shuffleboard to read for wrist
    SmartDashboard.putNumber("Wrist Current", m_wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("Wrist Position", getWristEncoderPosition());
  }

  private void configureWristMotor(){
    // Initializes motor and motor config
    m_wristMotor = new SparkFlex(canIDs.kWristCANID, MotorType.kBrushless);
    SparkFlexConfig wristMotorConfig = new SparkFlexConfig();

    // Sets configs
    wristMotorConfig.smartCurrentLimit(WristConstants.kSmartCurrentLimit);
    wristMotorConfig.idleMode(IdleMode.kBrake);

    // Set soft limits
    wristSoftLimitConfig = new SoftLimitConfig();
    wristSoftLimitConfig.forwardSoftLimit(WristConstants.kWristForwardSoftLimit);
    wristSoftLimitConfig.reverseSoftLimit(WristConstants.kWristReverseSoftLimit);
    wristSoftLimitConfig.forwardSoftLimitEnabled(true);
    wristSoftLimitConfig.reverseSoftLimitEnabled(true);

    wristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    wristMotorConfig.softLimit.reverseSoftLimitEnabled(true);
    wristMotorConfig.apply(wristSoftLimitConfig);

    // Initializes and sets up encoder for proper use
    wristEncoder = m_wristMotor.getAbsoluteEncoder();
    wristMotorConfig.absoluteEncoder.zeroOffset(WristConstants.kZeroOffset);
    wristMotorConfig.absoluteEncoder.inverted(true);
    wristMotorConfig.absoluteEncoder.positionConversionFactor(WristConstants.kPositionConversionFactor); // Test these
                                                                                                         // values
    wristMotorConfig.absoluteEncoder.velocityConversionFactor(WristConstants.kVelocityConversionFactor);

    // Sets up the Closed Loop Controller
    wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristMotorConfig.closedLoop.p(WristConstants.kWristP);
    wristMotorConfig.closedLoop.i(WristConstants.kWristI);
    wristMotorConfig.closedLoop.d(WristConstants.kWristD);
    wristMotorConfig.closedLoop.outputRange(WristConstants.kWristMinOutputPower, WristConstants.kWristMaxOutputPower);

    // Configures motor to desired configs
    m_wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void publishWristData(){
    wristData.AtSetpoint = atWristSetpoint(0);
    // CAG: this is part of the problem with needing an argument to check on the setpoint.
    m_stateTracker.setWristData(wristData);
  }
}
