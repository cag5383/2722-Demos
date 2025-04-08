// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDs;
import frc.robot.subsystems.StateTracker.ElevatorData;
import frc.robot.subsystems.StateTracker.StateTracker;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class Elevator extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  // Declares motors and motor configs
  private SparkFlex m_elevatorMotor;
  private SparkFlex m_followerElevatorMotor;

  // Declares encoder
  // private RelativeEncoder elevatorEncoder;

  // private double currentSetpoint;
  private SparkClosedLoopController elevator_pid;

  // Declares String Pot
  private AnalogPotentiometer elevatorStringPot;

  //Data structure periodically returned to the StateTracker.
  private StateTracker m_stateTracker;
  private ElevatorData elevatorData;


  public Elevator(StateTracker tracker) {
    m_stateTracker = tracker;
    configureLeadMotor();
    configureFollowMotor();
    configureStringPot();

    // Gets PID controller from motor
    elevator_pid = m_elevatorMotor.getClosedLoopController();
  }

  public void setElevatorMotorsSpeed(double inputSpeed) {
    // Sets the speed for the elevator to move freely
    // CAG: units are important here, because we can do it in
    // tangential speed, RPM, duty, volts, current, etc.
    // I'm also not sure this is really required?
    // Does anything use this besides runElevatorManually?
    // I think almost everything else should be using something PIDish?
    // Even if there is something else using this, is there a reason it
    // couldn't/shouldn't
    // use the runElevatorManually instead of setElevatorManually?

    m_elevatorMotor.set(inputSpeed);
  }

  public void runElevatorManually(double yElevatorSpeed) {
    // Controls the elevator manually with a joystick
    // currentSetpoint = -1;
    if (yElevatorSpeed >= .5)
      yElevatorSpeed = ElevatorConstants.kElevatorManualSpeed;
    if (yElevatorSpeed <= -.5)
      yElevatorSpeed = -0.5;

    m_elevatorMotor.set(yElevatorSpeed);
  }

  public void stopElevatorMotors() {
    // Stops the elevator
    m_elevatorMotor.stopMotor();
  }

  public void setElevatorPosition(double position) {
    // Moves the elevator to the desired position using PID and setpoint position
    // currentSetpoint = position;
    // CAG: May be worth clarifying in the name that it's a target/setpoint
    // - not resetting the measured position.
    elevator_pid.setReference(position, SparkFlex.ControlType.kPosition);
  }

  public double getStringPotPosition() {
    return m_elevatorMotor.getAnalog().getPosition();
  }

  public boolean atElevatorSetpoint(double inputElevatorPosition) {
    // Returns if the elevator is at the setpoint
    // CAG: Having to pass in the position I'm looking for is probably bad practice.
    // It means that any commands/subsystems that want to ask whether the elevator
    // is where it's supposed to be also have to know what to ask about.
    // You have to pass around and keep track of more information than is really
    // required.

    return Math.abs(inputElevatorPosition
        - m_elevatorMotor.getAnalog().getPosition()) < ElevatorConstants.kElevatorPositionTolerance;
  }

  public double getOutputCurrent() {
    return m_elevatorMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // CAG: coding standards - do we like get/set shuffleboards or is this okay?
    // Consistency is the name of the game.

    // Puts values on shuffleboard to read for elevator
    SmartDashboard.putNumber("Current of 1", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Current of 2", m_followerElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("String Pot Analog", m_elevatorMotor.getAnalog().getPosition());

    // CAG: I think it's really important long-term to figure out how to get systems
    // to maintain
    // setpoints using periodic calls instead of depending on commands.
  }

  private void configureLeadMotor() {
    // Initializes motors and motor configs
    m_elevatorMotor = new SparkFlex(canIDs.kElevatorCANID, MotorType.kBrushless);
    SparkFlexConfig mainElevatorConfig = new SparkFlexConfig();

    // Sets configs
    mainElevatorConfig.smartCurrentLimit(ElevatorConstants.kSmartCurrentLimit);
    mainElevatorConfig.idleMode(IdleMode.kBrake);

    // Sets up the Closed Loop Controller
    mainElevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAnalogSensor);
    mainElevatorConfig.closedLoop.pid(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI,
        ElevatorConstants.kElevatorD);
    mainElevatorConfig.closedLoop.outputRange(ElevatorConstants.kElevatorMinOutputPower,
        ElevatorConstants.kElevatorMaxOutputPower);

    // Set soft limits
    mainElevatorConfig.softLimit.forwardSoftLimitEnabled(true);
    mainElevatorConfig.softLimit.forwardSoftLimit(ElevatorConstants.kElevatorForwardSoftLimit);

    // Configures motors to desired configs
    m_elevatorMotor.configure(mainElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureFollowMotor() {
    m_followerElevatorMotor = new SparkFlex(canIDs.kFollowerElevatorCANID, MotorType.kBrushless);
    SparkFlexConfig followerElevatorConfig = new SparkFlexConfig();

    followerElevatorConfig.follow(canIDs.kElevatorCANID, true);

    // Sets configs
    followerElevatorConfig.idleMode(IdleMode.kBrake);

    // Apply final configs to follower
    m_followerElevatorMotor.configure(followerElevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void configureStringPot() {
    // Initializes string pot
    elevatorStringPot = new AnalogPotentiometer(ElevatorConstants.kStringPotPort,
        ElevatorConstants.kElevatorStringPotMaxRange, ElevatorConstants.kElevatorStringPotOffset);
  }

  private void publishElevatorData(){
    //First update the data
    elevatorData.AtSetpoint = atElevatorSetpoint(0);
    //CAG: this is part of the problem with having an argument for this.
    m_stateTracker.setElevatorData(elevatorData);
  }
}
