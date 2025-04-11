package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlinkIn.LEDAdapter;

/** An example command that uses an example subsystem. */
public class LEDsTimed extends Command {
  private final LEDAdapter m_led;
  private Timer m_timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LEDsTimed(LEDAdapter subsystem) {
    m_led = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Start the timer
    //Set the LEDs to a rainbow color.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Do you need to do anything else here if the LEDs have already been set to a color?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //when the command ends, set the LEDs to Red.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //How do we use the timer to check whether a certain amount of time has elapsed?
    return false;
  }
}