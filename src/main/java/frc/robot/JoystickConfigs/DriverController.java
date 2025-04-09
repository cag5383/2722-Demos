package frc.robot.JoystickConfigs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

public class DriverController {
    public static final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    //CAG: this is the list of all the possible buttons that we might want to use on an XBox Controller.
    //There's plenty of ways to use them, but generally they'll be either Triggers or Suppliers of some variety.
    //Triggers are true or false and can be used to Trigger commands
    //Suppliers can be used to pass along values - for example the coordinates of a joystick
    // or how pressed a trigger button is.
    //Suppliers can be used as inputs or passed to subsystems rather than triggering commands/events.

    //The button type (Trigger/Supplier) is declared with a desriptive name.
    public static Trigger AButton;
    public static Trigger XButton;
    public static Trigger YButton;
    public static Trigger BButton;
    public static BooleanSupplier LeftBumper;
    public static Trigger RightBumper;
    public static Trigger Start;
    public static Trigger Back;

    public static DoubleSupplier LeftStickX;
    public static DoubleSupplier LeftStickY;
    public static DoubleSupplier RightStickX;
    public static DoubleSupplier RightStickY;

    public static Trigger LeftTrigger;
    public static Trigger RightTrigger;

    public static Trigger DPadDown;
    public static Trigger DPadLeft;
    public static Trigger DPadUp;
    public static Trigger DPadRight;

    public DriverController(){
        //Buttons
        //CAG: Here we map the types and names to discrete behaviors on a controller.
        //Buttons are typically digital (true/false) inputs.
        // controller.a() returns whether that digital input is true (pressed) or false (unpressed)
        AButton = m_operatorController.a();
        XButton = m_operatorController.x();
        YButton = m_operatorController.y();
        BButton = m_operatorController.b();
        LeftBumper = m_operatorController.leftBumper();
        RightBumper = m_operatorController.rightBumper();
        Start = m_operatorController.start();
        Back = m_operatorController.back();

        //Sticks
        //Sticks and triggers are typically Analog inputs - they provide a range of inputs rather than 
        //true/false. For sticks, that range is typically represented as a coordinate between -1 and 1 in
        // the X and Y direction.
        //For a trigger button, that values is generally between 0 and 1.
        LeftStickX = () -> (m_operatorController.getLeftX());
        LeftStickY = () -> (m_operatorController.getLeftY());

        RightStickX = () -> (m_operatorController.getRightX());
        RightStickY = () -> (m_operatorController.getRightY());

        //Triggers
        //These triggers are set up to return a true/false when they are pressed more than a threshold.
        //In this case, more than 50% pressed.
        LeftTrigger = m_operatorController.leftTrigger(.5);
        RightTrigger = m_operatorController.rightTrigger(.5);

        //Direction Pad
        DPadDown =  m_operatorController.povDown();
        DPadLeft = m_operatorController.povLeft();
        DPadUp = m_operatorController.povUp();
        DPadRight = m_operatorController.povRight();
    }
}
