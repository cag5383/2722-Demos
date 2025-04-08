// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Wrist.*;
import frc.robot.commands.ConditionalSetElevatorPosition;
import frc.robot.commands.ConditionalSetWristPosition;
import frc.robot.commands.RunIntakeWithoutBeam;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialAlgaeIntakeUpperLevel extends SequentialCommandGroup {
  /** Creates a new SequentialAlgaeIntakeUpperLevel. */
  private final Wrist m_wrist;
  private final Intake m_intake;
  private final Elevator m_elevator;

  public SequentialAlgaeIntakeUpperLevel(Wrist wristInput, Intake intakeInput, Elevator elevatorInput) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_wrist = wristInput;
    m_intake = intakeInput;
    m_elevator = elevatorInput;

    addCommands(new ConditionalSetWristPosition(m_wrist, WristConstants.kWristElevatorSafeSetpoint));
    addCommands(new ConditionalSetElevatorPosition(m_elevator, ElevatorConstants.kElevatorUpperAlgaeRemove));
    addCommands(new ConditionalSetWristPosition(m_wrist, WristConstants.kWristAlgaeIntakeSetpoint));
    addCommands(new RunIntakeWithoutBeam(m_intake, IntakeConstants.kAlgaeIntakeSpeed));
  }
}
