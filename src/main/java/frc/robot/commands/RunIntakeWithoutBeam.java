// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/** An example command that uses an example subsystem. */
public class RunIntakeWithoutBeam extends Command {
  
  // Declare subsystem and speed variables
  private final Intake m_intake;
  private double speed;

  public RunIntakeWithoutBeam(Intake inputIntake, double inputSpeed) {
    // Initialize subsystem and speed variable from inputs
    m_intake = inputIntake;
    speed = inputSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run intake motor based on set speed
    m_intake.runIntakeMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop intake motor
    m_intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}