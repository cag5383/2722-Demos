package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveAdapter;
import frc.robot.subsystems.Vision.VisionAdapter;

/** An example command that uses an example subsystem. */
public class PIDYaw extends Command {
    private final VisionAdapter m_vision;
    private final DriveAdapter m_drive;
    private PIDController m_pid;

    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public PIDYaw(VisionAdapter vision, DriveAdapter drive) {
        m_vision = vision;
        m_drive = drive;

        // Initialize the PID Controller with the desired controller constants.
        m_pid = new PIDController(Kp, Ki, Kd);
        m_pid.setTolerance(0.1); // within 0.1 degrees of the target is good enough for now.

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //we're trying to get the yaw to be 0 degrees relative to the tag- completely squared up.
        m_pid.setSetpoint(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //don't do anything if there is no valid tag measurement.
        if (!m_vision.currTagObservation.validUpdate){
            return;
        }

        //Otherwise, there's a valid measurement.
        //Get the current yaw from the vision system.
        double currYaw = m_vision.currTagObservation.targetYaw; 

        //Calculate the controller response:
        double output = m_pid.calculate(currYaw);

        //Task the drivetrain
        m_drive.DriveTurn(output);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //We're done when the controller believes we are at its requested setpoint.
        return m_pid.atSetpoint();
    }
}