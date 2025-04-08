// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Wrist.Wrist;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class HoldWristElevatorWithOuttake extends Command {

  // Declares elevator subsystem and setpoint for the elevator position
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private Timer m_timer;

  private double eSP;
  private double wSP;
  
  public HoldWristElevatorWithOuttake(Elevator inputElevator, Wrist inputWrist, double elevatorSP, double wristSP, Intake inputIntake) {
    // Initializes the variables for the ConditionalSetElevatorPosition command
    m_elevator = inputElevator;
    m_wrist = inputWrist;
    m_intake = inputIntake;
    m_timer = new Timer();

    eSP = elevatorSP;
    wSP = wristSP;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_wrist, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the position of the elevator
    m_wrist.setWristPosition(wSP);
    m_elevator.setElevatorPosition(eSP);

    if (m_elevator.atElevatorSetpoint(eSP) && m_wrist.atWristSetpoint(wSP)) {
      if (!m_timer.isRunning()) {
        m_timer.start();
      }
      m_intake.runIntakeMotor(IntakeConstants.kCoralOuttakeSpeed);
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
    return m_timer.hasElapsed(0.5);
  }
}
