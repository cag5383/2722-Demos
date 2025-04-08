// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.Wrist;

/** An example command that uses an example subsystem. */
public class RunWristMotor extends Command {

  // Declares wrist subsystem and speed to be used to control wrist
  private final Wrist m_wrist;
  private double speed;
  
  public RunWristMotor(Wrist inputWrist, double inputSpeedOfWristMotor) {
    // Initializes the variables for the RunWristMotor command
    m_wrist = inputWrist;
    speed = inputSpeedOfWristMotor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the speed of the wrist motor
    m_wrist.setWristMotorSpeed(speed);
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
