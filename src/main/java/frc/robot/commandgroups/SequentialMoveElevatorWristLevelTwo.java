// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Wrist.*;
import frc.robot.commands.ConditionalSetElevatorPosition;
import frc.robot.commands.ConditionalSetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialMoveElevatorWristLevelTwo extends SequentialCommandGroup {
  /** Creates a new SequentialMoveElevatorWristLevelTwo. */
  private final Elevator m_elevator;
  private final Wrist m_wrist;

  public SequentialMoveElevatorWristLevelTwo(Elevator elevatorInput, Wrist wristInput) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevator = elevatorInput;
    m_wrist = wristInput;

    addCommands(new ConditionalSetWristPosition(m_wrist, WristConstants.kWristElevatorSafeSetpoint));
    addCommands(new ConditionalSetElevatorPosition(m_elevator, ElevatorConstants.kElevatorL2Setpoint));
    addCommands(new ConditionalSetWristPosition(m_wrist, WristConstants.kWristL2Setpoint));
  }
}
