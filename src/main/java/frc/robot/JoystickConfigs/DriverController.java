package frc.robot.JoystickConfigs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

public class DriverController {
    public static final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    //CAG:What are the things we want to be able to do/check?
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
    public static Trigger DPadDownLeft;
    public static Trigger DPadLeft;
    public static Trigger DPadUpLeft;
    public static Trigger DPadUp;
    public static Trigger DPadUpRight;
    public static Trigger DPadRight;
    public static Trigger DPadDownRight;

    public DriverController(){
        AButton = m_operatorController.a();
        XButton = m_operatorController.x();
        YButton = m_operatorController.y();
        BButton = m_operatorController.b();
        LeftBumper = m_operatorController.leftBumper();
        RightBumper = m_operatorController.rightBumper();
        Start = m_operatorController.start();
        Back = m_operatorController.back();

        //Sticks
        LeftStickX = () -> (m_operatorController.getLeftX());
        LeftStickY = () -> (m_operatorController.getLeftY());

        RightStickX = () -> (m_operatorController.getRightX());
        RightStickY = () -> (m_operatorController.getRightY());

        //Triggers
        //CAG: I wonder if there's a way to get these to be continuous inputs instead of discrete.
        //Right now it's true/false trigger based on whether the input is > threshold.
        LeftTrigger = m_operatorController.leftTrigger(.5);
        RightTrigger = m_operatorController.rightTrigger(.5);

        //CAG: I'm not sure I love this without testing how sensitive it is
        //How well can it distinguish down-left from left?
        //Maybe better to just poll the angle directly you can provide an angle).
        DPadDown =  m_operatorController.povDown();
        DPadDownLeft = m_operatorController.povDownLeft();
        DPadLeft = m_operatorController.povLeft();
        DPadUpLeft = m_operatorController.povUpLeft();
        DPadUp = m_operatorController.povUp();
        DPadUpRight = m_operatorController.povUpRight();
        DPadRight = m_operatorController.povRight();
        DPadDownRight = m_operatorController.povDownRight();
    }
}
