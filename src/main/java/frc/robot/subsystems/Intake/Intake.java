// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDs;
import frc.robot.subsystems.StateTracker.StateTracker;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // Declare motor and motor config
  private SparkFlex m_intakeMotor;

  // Declares Canrange(beambrake)
  private CANrange beamBraker;

  // Declares Reefcanrange
  private CANrange reefDetector;

  //StateTracker
  private final StateTracker m_stateTracker;

  public Intake(StateTracker tracker) {

    configureIntakeMotor();
    configureIntakeCANRange();
    configureReefCANRange();
    m_stateTracker = tracker;
  }

  public void runIntakeMotor(double inputSpeed) {
    // Set speed of intake motor
    m_intakeMotor.set(inputSpeed);
  }

  public void stopIntakeMotor() {
    // Stops the intake motor
    m_intakeMotor.stopMotor();
  }

  public boolean withinRange() {
    // Gets whether beambrake detects an object
    //CAG: it would be incredible if these names could be a little more descriptive.
    //e.g. SmartIntake beam break, etc.
    return beamBraker.getIsDetected().getValue();
  }

  public boolean detectReef() {
    // Gets whether beambrake detects an object
    return reefDetector.getIsDetected().getValue();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //CAG: Probably would want to poll CANRange for more than one frame, but that's a problem for later.
    if (this.withinRange() && StateTracker.elevatorData.AtSetpoint && StateTracker.wristData.AtSetpoint){
      //CAG: Score the coral.
    }


    //CAG: Coding standards - should we be using get/set shuffleboard?
    SmartDashboard.putNumber("Intake Current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Have Coral", withinRange());
    SmartDashboard.putBoolean("Aligned", detectReef());

  }

  private void configureIntakeMotor(){
    m_intakeMotor = new SparkFlex(canIDs.kIntakeMotorCANID, MotorType.kBrushless);
    SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

    // Set config for motor
    intakeMotorConfig.smartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
    intakeMotorConfig.idleMode(IdleMode.kBrake);

    m_intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureIntakeCANRange(){

    beamBraker = new CANrange(canIDs.kCANrangeID, IntakeConstants.kCANbus);
    CANrangeConfiguration beamBrakeConfig = new CANrangeConfiguration();

    //CAG: I would love notes about what all these paramters actually do for the sensor.
    //CAG: There's also a centerX and centerY -
    //do we know if there's a reason we didn't use those?
    FovParamsConfigs intakeConfigFOV = new FovParamsConfigs();
    intakeConfigFOV.FOVRangeX = IntakeConstants.kIntakeFOV;
    intakeConfigFOV.FOVRangeY = IntakeConstants.kIntakeFOV;
    
    //CAG: There's a MinSignalStrengthForValidMeasurement -
    //do we know if there's a specific reason we didn't use this?
    ProximityParamsConfigs intakeConfigProx = new ProximityParamsConfigs();
    intakeConfigProx.ProximityHysteresis = IntakeConstants.kIntakeProximityHysteresis;
    intakeConfigProx.ProximityThreshold = IntakeConstants.kIntakeProximityThreshold;

    beamBrakeConfig.withFovParams(intakeConfigFOV);
    beamBrakeConfig.withProximityParams(intakeConfigProx);

    beamBraker.getConfigurator().apply(beamBrakeConfig);
  }

  private void configureReefCANRange(){
    // Setting up Reefrange
    reefDetector = new CANrange(canIDs.kAlignerID, IntakeConstants.kCANbus);
    CANrangeConfiguration reefDetectorConfig = new CANrangeConfiguration();

    FovParamsConfigs reefConfigFov = new FovParamsConfigs();
    reefConfigFov.FOVRangeY = IntakeConstants.kReefFOV;
    reefConfigFov.FOVRangeX = IntakeConstants.kReefFOV;

    ProximityParamsConfigs reefConfigProx = new ProximityParamsConfigs();
    reefConfigProx.ProximityHysteresis = IntakeConstants.kReefHysteresis;
    reefConfigProx.ProximityThreshold = IntakeConstants.kReefMaxDistance;
    reefConfigProx.MinSignalStrengthForValidMeasurement = IntakeConstants.kReefSignalStrength;
    reefDetectorConfig.withFovParams(reefConfigFov);
    reefDetectorConfig.withProximityParams(reefConfigProx);

    reefDetector.getConfigurator().apply(reefDetectorConfig);
  }
}