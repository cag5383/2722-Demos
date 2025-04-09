
package frc.robot.subsystems.BlinkIn;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDAdapter extends SubsystemBase{
    private final BlinkIn m_blinkIn;

    public LEDAdapter() {
        m_blinkIn = new BlinkIn();
    }

    public void purple() {
        // Ask the Blinken for purple.
        m_blinkIn.purple();
    }

    public void red() {
        // Ask the blinken for red
        m_blinkIn.red();
    }

    public void green() {
        // Ask the blinken for green
        m_blinkIn.green();
    }

    public void blue() {
        // Ask the blinken for blue
        m_blinkIn.blue();
    }

    public void rainbow() {
                //Ask the blinken for rainbow rainbow
                //(As opposed to rainbow forest, etc)
        m_blinkIn.rainbowRainbowPalette();
    }
}
