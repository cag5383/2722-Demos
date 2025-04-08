// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ConditionalSetElevatorPosition;
import frc.robot.commands.RunElevatorMotorsCurrent;
import frc.robot.subsystems.Elevator.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorSmoothDown extends SequentialCommandGroup {
  /** Creates a new CoralIntake. */
  private final Elevator m_elevator;


  public ElevatorSmoothDown(Elevator elevatorInput) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevator = elevatorInput;
    
    addCommands(new ConditionalSetElevatorPosition(m_elevator, ElevatorConstants.kElevatorIntakeSetpoint));
    addCommands(new RunElevatorMotorsCurrent(m_elevator, -0.2));

  }
}
