// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Wrist.*;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveElevatorWristLevelTwo extends ParallelCommandGroup {
  /** Creates a new MoveElevatorWristLevelTwo. */
  private final Elevator m_elevator;
  private final Wrist m_wrist;

  public MoveElevatorWristLevelTwo(Elevator elevatorInput, Wrist wristInput) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevator = elevatorInput;
    m_wrist = wristInput;

    addCommands(new SetElevatorPosition(m_elevator, ElevatorConstants.kElevatorL2Setpoint));
    addCommands(new SetWristPosition(m_wrist, WristConstants.kWristL2Setpoint));
  }
}
