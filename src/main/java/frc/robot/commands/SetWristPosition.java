// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.Wrist;

/** An example command that uses an example subsystem. */
public class SetWristPosition extends Command {

  // Declares wrist subsystem and setpoint for the wrist position
  private final Wrist m_wrist;
  private double setpoint;
  
  public SetWristPosition(Wrist inputWrist, double inputSetpointOfWristMotor) {
    // Initializes the variables for the SetWristPosition command
    m_wrist = inputWrist;
    setpoint = inputSetpointOfWristMotor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the position of the wrist
    m_wrist.setWristPosition(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the wrist motor
    m_wrist.stopWristMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
