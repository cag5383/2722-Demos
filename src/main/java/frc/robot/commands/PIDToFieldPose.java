package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class PIDToFieldPose extends Command {

  // Declares elevator subsystem and speed to be used to control elevator
  private final DriveSubsystem m_DriveSubsystem;
  private double targetX;
  private double targetY;
  private double targetAngle;
  private PIDController pid_x;
  private PIDController pid_y;
  private PIDController pid_angle;
  
  public PIDToFieldPose(DriveSubsystem drive, Pose2d targetPose) {
    m_DriveSubsystem = drive;
    addRequirements(m_DriveSubsystem);

    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetAngle = targetPose.getRotation().getDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid_x = new PIDController(0, 0, 0);
    pid_x.setTolerance(0);
    pid_x.setSetpoint(targetX);

    pid_y = new PIDController(0, 0, 0);
    pid_y.setTolerance(0);
    pid_y.setSetpoint(targetY);

    pid_angle = new PIDController(0, 0, 0);
    pid_angle.setTolerance(0);
    pid_angle.setSetpoint(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = m_DriveSubsystem.getPose();
    double x_drive = pid_x.calculate(currPose.getX());
    double y_drive = pid_y.calculate(currPose.getY());
    double angle_drive = pid_angle.calculate(currPose.getRotation().getDegrees());

    m_DriveSubsystem.drive(x_drive, y_drive, angle_drive, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (pid_x.atSetpoint() && pid_y.atSetpoint() && pid_angle.atSetpoint());
  }
}
