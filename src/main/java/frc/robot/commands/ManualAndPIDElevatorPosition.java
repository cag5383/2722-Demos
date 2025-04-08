// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.*;

/** An example command that uses an example subsystem. */
public class ManualAndPIDElevatorPosition extends Command {

  // Declares elevator subsystem and setpoint for the elevator position
  private final Elevator m_elevator;
  private double setpoint;
  private double threshold;
  
  public ManualAndPIDElevatorPosition(Elevator inputElevator, double inputSetpointOfElevatorMotors) {
    // Initializes the variables for the ConditionalSetElevatorPosition command
    m_elevator = inputElevator;
    setpoint = inputSetpointOfElevatorMotors;
    threshold = 0.5;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the position of the elevator
    double difference = setpoint - m_elevator.getStringPotPosition();
    if (Math.abs(difference) <= threshold) {
      m_elevator.stopElevatorMotors();
      // m_elevator.setElevatorPosition(setpoint);
    } else {
      if (difference < 0) {
        m_elevator.setElevatorMotorsSpeed(ElevatorConstants.kElevatorMinOutputPower);
      } else {
        m_elevator.setElevatorMotorsSpeed(ElevatorConstants.kElevatorMaxOutputPower);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the elevator motors
    // m_elevator.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stops command when elevator is at setpoint
    return m_elevator.atElevatorSetpoint(setpoint);
  }
}
