
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveAdapter;
import frc.robot.subsystems.Vision.VisionAdapter;

/** An example command that uses an example subsystem. */
public class RotateUntilTag extends Command {
  private final DriveAdapter m_drive;
  private final VisionAdapter m_vision;


  //Notice that we are providing two subsystems to this command Constructor.
  //The Drive subsystem is what we will use to take action.
  //The Vision subsystem will be used to provide data.
  public RotateUntilTag(DriveAdapter drive, VisionAdapter vision) {
    m_drive = drive;
    m_vision = vision;

    //which, if any, of these subsystems should be required?
    //Remember - we typically only add a requirement when we want to make sure
    //that multiple commands are not trying to move the same subsystem.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Turn the robot either left or right.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //How can we check whether the vision system sees any apriltag?
    return false;
  }
}