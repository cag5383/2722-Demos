package frc.robot.subsystems.Drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.canIDs;

public final class DriveConstants {

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class ModuleConstants {
        // CAG: Did we check/test any of these? Or did it just kind of work?
        // CAG: It sure seems like we don't use many of these... Let's try to organize.

        // CAG These get used, either explicitly or implicitly, either here or in
        // Configs.java.
        public static final double kWheelDiameterMeters = 0.0811; // 0.0762
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

        // CAG:These do not. Should they? What's up here?
        public static final boolean kTurningEncoderInverted = true;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

    }

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    public static final double kMaxSpeedMetersPerSecond = 16;
    public static final double kMaxAngularSpeed = 8 * Math.PI;

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(29);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29);
    // Distance between front and back wheels on robot
    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            canIDs.kFrontLeftDrivingCanId,
            canIDs.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    public static final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            canIDs.kFrontRightDrivingCanId,
            canIDs.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    public static final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            canIDs.kRearLeftDrivingCanId,
            canIDs.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    public static final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            canIDs.kRearRightDrivingCanId,
            canIDs.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    public static final double kRobotRadius = kTrackWidth * Math.sqrt(2) / 2;

    // Pathplanner Holonomic PID Controller
    // CAG: An explanation here would be super cool:
    // When running paths, Pathplanner calculates a desired trajectory
    // Then while running it, it uses PID from frame to frame to try to
    // adjust velocities to stay "on course"
    // CAG: I don't think we've really tested these parameters to make sure they're
    // appropriate?

    public static final double kPAutoDrive = 3;
    public static final double kIAutoDrive = 5;
    public static final double kDAutoDrive = 1;
    public static final double kPturn = 3;
    public static final double kIturn = 5;
    public static final double kDturn = 1;

    public static final PPHolonomicDriveController pathplannerPID = new PPHolonomicDriveController(
            new PIDConstants(kPAutoDrive, kIAutoDrive, kDAutoDrive),
            new PIDConstants(kPturn, kIturn, kDturn));

    // CAG: Default path constraints for following paths.
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final PathConstraints defaultConstraints = new PathConstraints(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared,
            Units.degreesToRadians(180),
            Units.degreesToRadians(360));
}
