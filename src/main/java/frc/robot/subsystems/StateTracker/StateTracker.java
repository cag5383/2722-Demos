package frc.robot.subsystems.StateTracker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateTracker extends SubsystemBase{
    //Data structures to keep the results organized.
    public static ElevatorData elevatorData;
    public static WristData wristData;

    public StateTracker(){
        
    }

    @Override
    public void periodic(){

    }

    public void setWristData(WristData update){
        this.wristData = update;
    }

    public void setElevatorData(ElevatorData update){
        this.elevatorData = update;
    }
}
