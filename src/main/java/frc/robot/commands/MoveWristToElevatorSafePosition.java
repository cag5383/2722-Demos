// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.*;

/** An example command that uses an example subsystem. */
public class MoveWristToElevatorSafePosition extends Command {

  // Declares wrist subsystem and setpoint for the wrist position
  private final Wrist m_wrist;
  
  public MoveWristToElevatorSafePosition(Wrist inputWrist) {
    // Initializes the variables for the MoveWristToRest command
    m_wrist = inputWrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Moves Wrist to Rest Position
    m_wrist.setWristPosition(WristConstants.kWristElevatorSafeSetpoint);
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
