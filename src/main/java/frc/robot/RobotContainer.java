// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.JoystickConfigs.DriverController;
import frc.robot.JoystickConfigs.OperatorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.BlinkIn.BlinkIn;
import frc.robot.subsystems.Drive.*;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.StateTracker.StateTracker;
import frc.robot.subsystems.Tray.Tray;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.Wrist.*;
import frc.robot.commands.*;
import frc.robot.commandgroups.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final SendableChooser<Command> autoChooser;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final Tray m_tray;
    private final BlinkIn m_blinkIn;
    private final Vision m_vision;
    private final StateTracker m_StateTracker;

    private Command m_lastCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_StateTracker = new StateTracker();
        m_elevator = new Elevator(m_StateTracker);
        m_wrist = new Wrist(m_StateTracker);
        m_intake = new Intake(m_StateTracker);
        m_tray = new Tray();
        m_blinkIn = new BlinkIn();
        m_vision = new Vision();

        // Configure the button bindings
        configureDriverBindings();
        configureOperatorBindings();
        registerNamedCommands();

        // CAG: I'm assuming this adds the drop down to choose an auto, but we should
        // clarify/specify.
        // I don't want this to turn into something that's just here because it works
        // but newer students don't know what it does.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous", autoChooser);

    }

    private void configureDriverBindings() {

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                DriverController.LeftStickY.getAsDouble()
                                        * Math.abs(DriverController.LeftStickY.getAsDouble()),
                                DriverController.LeftStickX.getAsDouble()
                                        * Math.abs(DriverController.LeftStickX.getAsDouble()),
                                DriverController.RightStickX.getAsDouble()
                                        * Math.abs(DriverController.RightStickX.getAsDouble()),
                                true,
                                DriverController.LeftBumper.getAsBoolean()),
                        m_robotDrive));

        DriverController.XButton.whileTrue(new RunCommand(
                () -> m_robotDrive.setX(), m_robotDrive));

        DriverController.AButton.whileTrue(m_robotDrive.pathfindtoPath("Deep Mid", DriveConstants.defaultConstraints));
        // CAG: I'm not going to try all the DPad reef commands since, but here's an
        // example of something we might be able to do.
        DriverController.DPadRight.and(DriverController.DPadDownRight)
                .whileTrue(m_robotDrive.pathfindtoPath("B", DriveConstants.defaultConstraints));

        // CAG: Do we use this? If not, I'm not sure why it's here.
        // If yes, I worry about how it interacts with odometry/pose and vision stuff
        // that also depends on heading.
        DriverController.XButton.whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
    }

    private void configureOperatorBindings() {

        m_elevator
                .setDefaultCommand(new RunCommand(
                        () -> m_elevator.runElevatorManually(-OperatorController.RightStickY.getAsDouble()
                                * Math.abs(OperatorController.RightStickY.getAsDouble())),
                        m_elevator));

        m_wrist.setDefaultCommand(new RunCommand(() -> m_wrist.runWristManually(
                -OperatorController.LeftStickY.getAsDouble()
                        * Math.abs(OperatorController.LeftStickY.getAsDouble())),
                m_wrist));

        OperatorController.AButton.whileTrue(new CoralIntake(m_elevator, m_wrist, m_intake));
        OperatorController.XButton.whileTrue(new SequentialMoveElevatorWristLevelTwo(m_elevator, m_wrist));
        OperatorController.YButton.whileTrue(new SequentialMoveElevatorWristLevelThree(m_elevator, m_wrist));
        OperatorController.BButton.whileTrue(new SequentialMoveElevatorWristLevelFour(m_elevator, m_wrist));
        // CAG: I don't know which DPad direction we're using for this, but hopefully
        // this is okay.
        OperatorController.DPadLeft.whileTrue(new MoveToRestFromL4(m_elevator, m_wrist));

        OperatorController.LeftBumper.whileTrue(new SequentialAlgaeIntakeUpperLevel(m_wrist, m_intake, m_elevator));
        OperatorController.LeftTrigger.whileTrue(new SequentialAlgaeIntakeLowerLevel(m_wrist, m_intake, m_elevator));

        OperatorController.Start.whileTrue(new MoveElevatorWristProcessor(m_elevator, m_wrist, m_intake));
        OperatorController.RightBumper.whileTrue(new MoveToRest(m_elevator, m_wrist));

        OperatorController.RightTrigger
                .whileTrue(new RunIntakeWithoutBeam(m_intake, IntakeConstants.kCoralOuttakeSpeed));
        OperatorController.DPadDown.whileTrue(new RunIntakeWithoutBeam(m_intake, -0.20));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void periodic() {
        // CAG: Command scheduler is run at the robot level and only calls subsystem
        // periodics by default
        // We did this at the periodic level because it helps avoid passing extra
        // subsystems.
        // Otherwise, there's not an obvious place that knows current state of elevator
        // + intake + wrist.
        // Maybe there should be some kind of state tracker that does that?
        // Regardless, if this approach is used, RobotContainer.periodic() has to be
        // added after CommandScheduler
        // in Robot.java RobotPeriodic.

        //CAG: publish pose to Limelights so they can do megatag stuff.
        m_vision.setPose(m_robotDrive.getPose());
        m_vision.setYawRate(m_robotDrive.getTurnRate()); //CAG: do the units work? I think Megatag wants degrees/second.

        if (m_intake.detectReef() && m_elevator.getStringPotPosition() > ElevatorConstants.kElevatorRestSetpoint) {
            m_blinkIn.purple();
        } else if (m_wrist.atWristSetpoint(WristConstants.kWristIntakeSetpoint)
                && m_elevator.atElevatorSetpoint(ElevatorConstants.kElevatorRestSetpoint) && m_intake.withinRange()) {
            m_blinkIn.white();
        } else if (m_wrist.atWristSetpoint(WristConstants.kWristIntakeSetpoint)
                && m_elevator.atElevatorSetpoint(ElevatorConstants.kElevatorRestSetpoint)) {
            m_blinkIn.green();
        } else {
            m_blinkIn.orange();
        }
    }

    private void registerNamedCommands() {
        // Registering named commands makes them available in Pathplanner for building
        // autos/paths.

        // Reef Level Elevator + Wrist Commands
        NamedCommands.registerCommand("SL4", new SequentialMoveElevatorWristLevelFour(m_elevator, m_wrist));
        NamedCommands.registerCommand("RL4", new MoveToRestFromL4(m_elevator, m_wrist));
        NamedCommands.registerCommand("SL2", new SequentialMoveElevatorWristLevelTwo(m_elevator, m_wrist));
        NamedCommands.registerCommand("WIntake",
                new ConditionalSetWristPosition(m_wrist, WristConstants.kWristIntakeSetpoint));
        NamedCommands.registerCommand("L4Maintain",
                new SetElevatorPosition(m_elevator, ElevatorConstants.kElevatorL4Setpoint));

        // Intake Pathplanner Commands
        // "IntakeCoral" is conditional
        NamedCommands.registerCommand("IntakeCoral", new RunIntakeMotor(m_intake, IntakeConstants.kCoralIntakeSpeed));

        // "IntakeAlgae" is conditional
        NamedCommands.registerCommand("IntakeAlgae",
                new RunIntakeWithoutBeam(m_intake, IntakeConstants.kAlgaeIntakeSpeed));

        // Outtake Pathplanner Commands
        NamedCommands.registerCommand("OuttakeCoral",
                new RunIntakeWithoutBeam(m_intake, IntakeConstants.kCoralOuttakeSpeed));
        // CAG: Are we ever going to outtake algae as part of an auto/path?
        NamedCommands.registerCommand("OuttakeAlgae",
                new RunIntakeWithoutBeam(m_intake, IntakeConstants.kAlgaeOuttakeSpeed));

        // Tray Pathplanner Commands
        NamedCommands.registerCommand("TMove", new SetTrayPosition(m_tray, 20));

        // PosetoPose Commands
        // CAG: we used pathplanner to make consistent paths for approaching scoring
        // positions labeled A-I
        // These commands will pathfind to the beginning of those paths and then follow
        // them to scoring positions.
        NamedCommands.registerCommand("A", m_robotDrive.pathfindtoPath("A", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("B", m_robotDrive.pathfindtoPath("B", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("C", m_robotDrive.pathfindtoPath("C", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("D", m_robotDrive.pathfindtoPath("D", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("E", m_robotDrive.pathfindtoPath("E", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("F", m_robotDrive.pathfindtoPath("F", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("G", m_robotDrive.pathfindtoPath("G", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("H", m_robotDrive.pathfindtoPath("H", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("I", m_robotDrive.pathfindtoPath("I", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("J", m_robotDrive.pathfindtoPath("J", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("K", m_robotDrive.pathfindtoPath("K", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("L", m_robotDrive.pathfindtoPath("L", DriveConstants.defaultConstraints));
        NamedCommands.registerCommand("driveForward", new DriveForward(m_robotDrive));
    }
}