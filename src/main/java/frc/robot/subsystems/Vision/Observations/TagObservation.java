package frc.robot.subsystems.Vision.Observations;

import edu.wpi.first.math.geometry.Pose2d;

public class TagObservation {
        //CAG: I kind of want these to be rolling averages?
    //Maybe that's a later problem.
    public double targetX;
    public double targetY;
    public double targetYaw;
    public double ambiguity;
    public int tagID;
    public boolean validUpdate;
    public String source;

    public TagObservation(){
        //CAG: Initialize to invalid.
        targetX = 0;
        targetY = 0;
        targetYaw = 0;
        ambiguity = 0;
        validUpdate = false;
        tagID = -1;
        source = "";
    }

}
