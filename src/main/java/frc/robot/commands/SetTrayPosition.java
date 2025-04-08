// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tray.Tray;

/** An example command that uses an example subsystem. */
public class SetTrayPosition extends Command {

  // Declares tray subsystem and setpoint for the tray position
  private final Tray m_tray;
  private double setpoint;
  
  public SetTrayPosition(Tray inputTray, double inputSetpointOfTray) {
    // Initializes the variables for the SetTrayPosition command
    m_tray = inputTray;
    setpoint = inputSetpointOfTray;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tray);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the position of the tray
    m_tray.setTrayPosition(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the tray motor
    m_tray.stopTrayMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
