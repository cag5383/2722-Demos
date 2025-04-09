package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveAdapter extends SubsystemBase {

    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    public DriveAdapter() {
        configureInitialPosition();
    }


    public void DriveForward(double speed) {
        // Tells the drivetrain to drive forward or backward based on whether the speed
        // is positive or negative.
        m_robotDrive.drive(speed, 0, 0, false, true);
    }

    public void DriveStrafe(double speed) {
        // Tells the drivetrain to drive left or right based on whether the speed is
        // positive or negative.
        m_robotDrive.drive(0, speed, 0, false, true);
    }

    public void DriveTurn(double speed) {
        // Tells the drivetrain to drive forward or backward based on whether the speed
        // is positive or negative.
        m_robotDrive.drive(0, 0, speed, false, true);
    }

    public void driveRobotRelative(double forwardSpeed, double strafeSpeed, double rotSpeed) {
        m_robotDrive.drive(forwardSpeed, strafeSpeed, rotSpeed, false, true);
    }

    public void driveFieldRelative(double forwardSpeed, double strafeSpeed, double rotSpeed) {
        m_robotDrive.drive(forwardSpeed, strafeSpeed, rotSpeed, true, true);
    }

    public void configureInitialPosition() {

        //DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
        //DriverStation.getAlliance().get() == DriverStation.Alliance.Red

        Rotation2d startRot = new Rotation2d(0);
        Pose2d startPose = new Pose2d(0,0, startRot);

        m_robotDrive.resetOdometry(startPose);

    }

}
