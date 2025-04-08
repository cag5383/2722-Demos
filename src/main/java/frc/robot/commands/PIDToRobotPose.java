package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class PIDToRobotPose extends Command {

  // Declares elevator subsystem and speed to be used to control elevator
  private final DriveSubsystem m_DriveSubsystem;
  private double targetX;
  private double targetY;
  private double targetYaw;
  private PIDController pid_x;
  private PIDController pid_y;
  private PIDController pid_yaw;
  
  public PIDToRobotPose(DriveSubsystem drive, double tX, double tY, double tYaw) {
    m_DriveSubsystem = drive;
    addRequirements(m_DriveSubsystem);

    //tX, tY, and tYaw represent an X, Y, and yaw of the robot relative to the target tag.
    //Hypothetically we could pass in a particular tag too? IDK if we need to get that sophisticated though.

    targetX = tX;
    targetY = tY;
    targetYaw = tYaw;
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

    pid_yaw = new PIDController(0, 0, 0);
    pid_yaw.setTolerance(0);
    pid_yaw.setSetpoint(targetYaw);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Query the vision system for targetpose_robotspace from the network table.
    //This gives us the position/orientation of the "primary" in-view apriltag relative to the robot.
    //It'll use the limelight configuration data to determine where the robot is.

    //The vision code would be something like:
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray()

    //Placeholder sample data:
    double visionData[] = {0,0,0,0,0,0}; //X,Y,Z,pitch,yaw,roll per limelight docs.
    double x = visionData[0];
    double y = visionData[1];
    double yaw = visionData[4];

    double x_drive = pid_x.calculate(x);
    double y_drive = pid_y.calculate(y);
    double angle_drive = pid_yaw.calculate(yaw);

    //CAG: I don't know if we know whether this works when we say field relative is false.
    //I guess there's only one way to find out.
    m_DriveSubsystem.drive(x_drive, y_drive, angle_drive, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (pid_x.atSetpoint() && pid_y.atSetpoint() && pid_yaw.atSetpoint());
  }
}
