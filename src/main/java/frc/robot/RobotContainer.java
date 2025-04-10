// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.JoystickConfigs.DriverController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.BlinkIn.BlinkIn;
import frc.robot.subsystems.BlinkIn.LEDAdapter;
import frc.robot.subsystems.Drive.*;
import frc.robot.subsystems.Vision.*;
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
        private final DriveAdapter m_robotDrive = new DriveAdapter();
        private final SendableChooser<Command> autoChooser;
        private final LEDAdapter m_led;
        private final VisionAdapter m_vision;

        // CAG: set to true for SwerveBasics example:
        private boolean usingSwerveDefaults = false;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_led = new LEDAdapter();
                m_vision = new VisionAdapter();

                // Configure the button bindings
                configureDriverBindings();
                registerNamedCommands();

                // CAG: I'm assuming this adds the drop down to choose an auto, but we should
                // clarify/specify.
                // I don't want this to turn into something that's just here because it works
                // but newer students don't know what it does.
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autonomous", autoChooser);

        }

        private void configureDriverBindings() {

                // CAG: an example of how to map a command to the A button on the driver
                // controller.
                // When the button is pressed, the LEDs should change green.
                DriverController.AButton.onTrue(new RunCommand(() -> m_led.green()));                

                // CAG: an example of how to map a command to the DPad on the driver controller.
                // While DPad up is pressed, the robot should drive forward with speed 0.5.
                DriverController.DPadUp.whileTrue(new RunCommand(() -> m_robotDrive.DriveForward(.5), m_robotDrive));



                // Configure default commands
                //Default commands avoid the need for a trigger that 'activates' the command.
                //The subsystem assumes the command should be running unless something else is using the subsystem.
                if (usingSwerveDefaults){
                        m_robotDrive.setDefaultCommand(
                        new RunCommand(
                                () -> m_robotDrive.driveRobotRelative(
                                        DriverController.LeftStickY.getAsDouble(),
                                        DriverController.LeftStickX.getAsDouble(),
                                        DriverController.RightStickX.getAsDouble()),
                        m_robotDrive));
                  }

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

                //When a tag observation is created, the tagID is initialized to -1 and only updated to a positive value
                //if a valid target is observed.
                if (m_vision.currTagObservation.tagID == -1){
                        m_led.red();
                }
        }

        private void registerNamedCommands() {
        }
}