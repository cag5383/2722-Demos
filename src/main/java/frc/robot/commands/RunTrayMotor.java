// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tray.Tray;

/** An example command that uses an example subsystem. */
public class RunTrayMotor extends Command {

  // Declares wrist subsystem and speed to be used to control tray
  private final Tray m_tray;
  private double speed;
  
  public RunTrayMotor(Tray inputTray, double inputSpeedOfTray) {
    // Initializes the variables for the RunTrayMotor command
    m_tray = inputTray;
    speed = inputSpeedOfTray;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tray);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the speed of the tray motor
    m_tray.setTrayMotorSpeed(speed);
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
