// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.BlinkIn;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;

public class BlinkIn extends SubsystemBase {

    private Spark blinkinController;
    private Timer m_timer;
    private boolean timerOn = false;

    public BlinkIn() {
        blinkinController = new Spark(BlinkInConstants.kPwmPort);
        m_timer = new Timer();  
    }

    public void set(double value) {
        blinkinController.set(value);

    }

    public void startTimer() {
        m_timer.restart();
        timerOn = true;
    }

    
    public void periodic() {
        // if (timerOn) {
        //     if (m_timer.hasElapsed(BlinkInConstants.kTimerDuration)) {
        //         ReturnToDefault();
        //     }
        //  }
    }

    // }
    public void ReturnToDefault() {

        m_timer.stop();
        timerOn = false;
        red();
    }

    public void purple() {
        set(0.89);
    }

    public void red() {
        set(0.61);
    }

    public void orange() {
        set(0.65);
    }

    public void yellow() {
        set(0.69);
    }

    public void green() {
        set(0.77);
    }

    public void blue() {
        set(0.87);
    }

    public void white() {
        set(0.93);
    }

    public void heartbeatRed() {
        set(-0.25);
    }
    public void heartbeatBlue(){
        set(-0.23);
    }
    public void rainbowForestPalette() {
        set(-0.91);
    }

    public void rainbowRainbowPalette() {
        set(-0.99);
    }

    public void twinklesRainbowPalete() {
        set(-0.55);
    }

    public void wavesRainbowPalette() {
        set(-0.45);
    }

    public void black(){
        set(0.99);
    }
}