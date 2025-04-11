
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveAdapter;

public class DriveForwardTimed extends Command {
  private final DriveAdapter m_drive;
  private Timer m_timer = new Timer();  



  //Notice that the command Constructor needs a list of subsystems that it will use as inputs.
  //The systems passed in are stored in local variables.
  //the input "drive" is only defined in the constructor.
  //Once the reference has been stored in m_drive, we can continue to use it everywhere else.
  public DriveForwardTimed(DriveAdapter drive) {
    
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    // you cannot have multiple active commands that Require the same subsystem.
    // Consequently, we should be careful that we only use addRequirements to
    //require subsystems with parts that we intend to move.

    //For example, if I intend to use a command to drive the robot, I should add the drive subsystem as a requirement.
    //This is because I would not want multiple commands trying to drive the robot at once.

    //However, I might want information from another subsystem, such as Vision.
    //If all I am using a system for is for information, it may not need to be a Requirement.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  //On initialization, start the timer.
  @Override
  public void initialize() {
    m_timer.restart();
}

  // Called every time the scheduler runs while the command is scheduled.
  //While executing, drive forward at a fixed speed
  @Override
  public void execute() {
    m_drive.DriveForward(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //returns true or false depending on whether the timer has been running for more than 2 seconds.
    return (m_timer.hasElapsed(2));
  }
}