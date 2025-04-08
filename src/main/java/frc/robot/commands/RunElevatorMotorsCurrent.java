// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

/** An example command that uses an example subsystem. */
public class RunElevatorMotorsCurrent extends Command {

  // Declares elevator subsystem and speed to be used to control elevator
  private final Elevator m_elevator;
  private double speed;
  
  public RunElevatorMotorsCurrent(Elevator inputElevator, double inputSpeedOfElevatorMotors) {
    // Initializes the variables for the RunElevatorMotors command
    m_elevator = inputElevator;
    speed = inputSpeedOfElevatorMotors;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the speed of the elevator motors
    m_elevator.setElevatorMotorsSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the elevator motors
    m_elevator.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getOutputCurrent() > 35;
  }
}
