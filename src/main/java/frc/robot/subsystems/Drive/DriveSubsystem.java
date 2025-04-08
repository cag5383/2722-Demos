// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

//CAG: I think we should re-consider using some kind of 
//Slew limiter or motion profiling - this would be to help avoid tipping.
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.IOException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private MAXSwerveModule m_frontLeft = DriveConstants.m_frontLeft;
  private MAXSwerveModule m_frontRight = DriveConstants.m_frontRight;
  private MAXSwerveModule m_rearLeft = DriveConstants.m_rearLeft;
  private MAXSwerveModule m_rearRight = DriveConstants.m_rearRight;

  // The gyro sensor
  // CAG: we could probably add a boolean toggle to swap between pigeon/imu
  // private final Pigeon2 m_gyro = new Pigeon2(13, "ChargeCAN");
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private Field2d m_field = new Field2d();

  // CAG: Comments explaining why we need different starting poses would be
  // helpful.
  // We know that orientation is probably the most important part of this, but
  // don't say that anywhere.
  // We could also use the same check that pathplanner does for alliance instead
  // of recalculating.
  // This is also a reasonable candidate for using a method to make this more
  // readable.

  private Pose2d allianceStartPoseBlue;
  private Pose2d allianceStartPoseRed;

  private SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      // CAG: what settings is this pulling? From where?
      // Do we know if they should change from year to year or whether they're
      // accurate/relevant?
      // I assume this is pulling the kinematic data from pathplanner, but
      // comments/clarification would help.

      // CAG: Comments explaining what these controller constants are for
      // and how/if they were identified would be super helpful.
      // Sometimes it's obvious (e.g. Elevator) because there's only one thing.
      // But a little history/clarity helps to keep track of loose ends.
      // For example, I don't think we ever tested/tuned these
      // My recollection is they kind of worked okay enough so we moved on,
      // But these are one of a couple things that contribute to variance in
      // pathplanner behaviors.
      // (Along with odomoetry, telemtry, vision ambiguity, etc)

      // CAG: How much do we know about how each of these things get used in
      // pathplanner
      // And whether they're correct or what the effect of them being wrong is?
      // e.g. I'm not sure how/whether feedforward is defined/necessary?
      // Kturn is not particularly descriptive,
      // especially since both sets of controls are for auto but only one specifies.
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                                // ChassisSpeeds
          DriveConstants.pathplannerPID, // HolonomicPathFollowerConfig, this should likely live in your
          // Constants class
          // Drive base radius in meters. Distance from robot center to
          // furthest module.
          // Default path replanning config. See the API for the options here
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            // CAG: we also care about whether we're Red/Blue. Seems like something we could
            // determine outside of this giant declaration.
            // Generally curious how much of this giant block we understand
            // versus copied from a pathplanner example.
            System.out.println("Alliance is Present");
            var alliance = DriverStation.getAlliance(); // Optional<Alliance>
            if (alliance.isPresent()) {
              System.out.println("Alliance is Present" + alliance.get());
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false; // return selectedalliance;
            // red = true blue = false
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {

      e.printStackTrace();
    }
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));

    m_gyro.reset();

    // Pose Estimator for tracking robot pose
    //CAG: this correlates the current gyro measurement and swerve positions
    // to the specified starting pose for field centric driving/pathfinding.
    Pose2d startPose = getStartPose();
    this.m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroscopeRotation(),
        getModulePositions(),
        startPose);

    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    });
  }

  @Override
  public void periodic() {

    // CAG: Comment would help - how much do we know about how it's integrating
    // The module positions and computing actual position?
    // Did we do any testing to see how reasonable the odometry is without help from
    // Vision?
    // that matters when we can't see tags or the measurements from the tags are
    // uncertain.
    // And may be contributing to some of our pathplanner/pose uncertainty.
    // Update the pose estimator with physical odometry measurements.
    m_poseEstimator.update(
        getGyroscopeRotation(),
        getModulePositions());

    getVisionUpdates();

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    setShuffleBoard();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {

    // CAG:We give this to pathplanner and never use it otherwise.
    // do we know when/why it uses it?
    // CAG: we dont have to do this on initialization if we're careful about
    //when/how we initialize the pose estimator.

    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = { m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState() };
    return states;
  }

  public ChassisSpeeds getSpeeds() {
    // CAG: do we understand the math this is doing and
    // where it gets used?
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean slowMode) {
    // CAG: How much of this math do we actually understand?
    // Have we tried using this to do robot-relative driving or only field-relative?
    // This may have been helpful when we briefly tried to do robot-relative
    // controls
    // for reef alignment - I think there was another command we tried using that
    // didnt really work.

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    if (slowMode) {
      xSpeedDelivered = xSpeed * (DriveConstants.kMaxSpeedMetersPerSecond / 4);
      ySpeedDelivered = ySpeed * (DriveConstants.kMaxSpeedMetersPerSecond / 4);
      rotDelivered = rot * (DriveConstants.kMaxAngularSpeed / 4);
    } else {
      xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // CAG: should this use setModuleStates? Or should we get rid of
    // setModuleStates?
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    if (slowMode) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, (DriveConstants.kMaxSpeedMetersPerSecond / 4.0));
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    // CAG: should this use setModuleStates? Or should we get right of
    // setModuleStates?
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
//This gets used by the pathplanner auto builder to run drive commands.
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }


  public double getHeading() {
    return m_gyro.getAngle();
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // return m_gyro.getAngularVelocityZWorld().getValueAsDouble() *
    // (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return m_gyro.getRate();
  }

  public Command pathfindToPose(Double xposition, Double yposition, Rotation2d rotationdeg, PathConstraints constraints) {

    // CAG: Some information here about what pathplanner is doing under the hood
    // would be super helpful.
    // What kind of command gets returned and how/why does it end?
    // I think we should pass in path constraints as an argument so this is more
    // flexible.
    // You can still have a "SlowPathfind"/FastPathfind so that you don't have to
    // keep constraints speeds
    // All the time, but it would give us the option to handle them differently,
    // which we don't have right now.

    Pose2d targetPose = new Pose2d(xposition, yposition, rotationdeg);
    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    return pathfindingCommand;
  }

  private void getShuffleBoard() {

  }

  private void setShuffleBoard() {

    Pose2d currentPose = getPose();

    // CAG: Put the rest of pose to dashboard
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("PoseX", currentPose.getX());
    SmartDashboard.putNumber("PoseY", currentPose.getY());
    SmartDashboard.putNumber("PoseYaw", currentPose.getRotation().getDegrees());
    SmartDashboard.putData("field", m_field);

    // CAG: Pretty sure this puts a ton of subsystem info on the dashboard - such as
    // What the command(s) the system is running. Not sure how computationally
    // intensive it is.
    // SmartDashboard.putData(this);
  }

  public Command pathfindtoPath(String pathdest,PathConstraints constraints) {
    // CAG: Some information here about what pathplanner is doing under the hood
    // would be super helpful.
    // What kind of command gets returned and how/why does it end?
    // I think we should pass in path constraints as an argument so this is more
    // flexible.
    // You can still have a "SlowPathfind"/FastPathfind so that you don't have to
    // keep constraints speeds
    // All the time, but it would give us the option to handle them differently,
    // which we don't have right now.

    // It's also possible that we'd want to store information about the path we
    // expect to be following, if possible.
    // Things like start/stop point, etc might let us start/end commmands based on
    // things like
    // how close we are to the end point, etc.
    // Right now we just have to trust that Pathplanner will get us close and then
    // stop -
    // we could check that work and force the correction.

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathdest);

      // CAG: I think we don't need this last_command stuff unless we're trying to
      // store
      // Specific information about a "current" command.
      // We're definitely not checking very carefully whether the command is still
      // active
      // or using any information from it, so I'm not sure it's necessary right now,
      // but I can imagine us doing useful things with this kind of information.

      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    catch (IOException e) {
      System.out.println("Pathfind to pose - IO Exception, File can not be read");
    } catch (FileVersionException e) {
      System.out.println("Pathfind to pose -File Version Exception, File version does not match expected");
    } catch (Exception e) {
      System.out.println("Pathfind to pose - Unknown Exception");
    }

    return null;
  }

  Pose2d getStartPose() {

    // CAG: In order for field-centric driving/pathfinding to work,
    // We have to give an initial pose that makes sense as an initial reference.
    // The most important part is likely the orientation, since that feeds
    // MegaTag2 pose estimates.

    // CAG: We've seen that MegaTag2 uses rotation to choose between 2 likely
    // solutions -
    // If the orientation is off by too much, it will place the robot in a very
    // wrong position.

    allianceStartPoseBlue = new Pose2d(new Translation2d(7.303, 4.035),
        new Rotation2d(Units.degreesToRadians(0)));
    allianceStartPoseRed = new Pose2d(new Translation2d(8, 2),
        new Rotation2d(Units.degreesToRadians(180)));

    return ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? allianceStartPoseBlue
        : allianceStartPoseRed);
  }

  void getVisionUpdates() {
    // CAG: Would love to see the vision updates in a method and clearly annotated.
    // We should also think about how we want pose estimation/vision to work.
    // Do we want a pose estimation subsystem that combines all the measurements
    // rather than
    // The drivetrain getting more and more complex every time we add more vision
    // inputs?
    // Do we want a vision subsystem that queries all the cameras and processes all
    // the inputs?
    // or are we comfortable with that living in the pose estimator/drivetrain?
    // If we want other commands/subsystems to be able to be aware of pose info
    // without
    // Needing to actually drive the subsystem, pulling that information out may
    // make sense.
    boolean doRejectUpdate = false;

    //CAG: this sets the robot orientation for MegaTag2 - it uses the current orientation to solve for
    //where the robot is.
    //CAG: We should probably specify yaw rate at least?
    //Most of the other ones should be zero unless something is going wrong (i.e. collisions)
    //But we can get yaw rate and it should matter...
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    
    doRejectUpdate = Math.abs(getTurnRate()) > 720 || // reject the update if we're turning too fast to get a good measurement.
    mt2 == null ||  // reject the update if the limelight is still booting.
    mt2.tagCount == 0; // reject the update if there are no tags visible.
      // CAG: Other teams have additional/more complicated ways to reject vision
      // measurements.
      // Things like: do the poses return something "in-bounds"/on the field.
      // Is there too much "ambiguity" in the measurement - this is something that
      // Photon Vision
      // returns explicitly - I think limelight does something different, but we'd
      // have to dig
      // into the raw data in network tables instead of the neatly packaged stuff
      // we're doing now.
      // Do we know how this behaves when it sees multiple tags or are we banking on
      // it behaving nicely?

    if (!doRejectUpdate) {
      // CAG: We never validated these standard deviations.
      // They represent confidence in the vision measurements (low is good, high is
      // bad.)
      // 2539 calculates a linear relationship between distance from the tag and confidence 
      //in the measurement.
      //Peddie does rolling averages to try smooth some of the measurements.
      //Do we have a good idea of how un/stable the measurements are?
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }
  }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
      //CAG: do we ever do this besides on startup?
      //Should we ever want to do this, even during startup?
      //It doesn't look like it's ever called.
      m_gyro.reset();
      //CAG: if we do this, I think we have to do this also:
      resetOdometry(this.getPose());
    }
}