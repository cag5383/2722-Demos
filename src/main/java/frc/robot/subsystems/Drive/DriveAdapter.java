package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveAdapter extends SubsystemBase{

    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    public void DriveForward(double speed){
        //Tells the drivetrain to drive forward or backward based on whether the speed is positive or negative.
        m_robotDrive.drive(speed, 0, 0, false, true);
    }

    public void DriveStrafe(double speed){
        //Tells the drivetrain to drive left or right based on whether the speed is positive or negative.
        m_robotDrive.drive(0, speed, 0, false, true);
    }

    public void DriveTurn(double speed){
        //Tells the drivetrain to drive forward or backward based on whether the speed is positive or negative.
        m_robotDrive.drive(0, 0, speed, false, true);
    }
}
