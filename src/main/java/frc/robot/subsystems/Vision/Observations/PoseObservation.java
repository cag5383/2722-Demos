package frc.robot.subsystems.Vision.Observations;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseObservation {
    //CAG: I kind of want these to be rolling averages?
    //Maybe that's a later problem.
    public Pose2d pose;
    public double timestamp;
    public double stdDevX;
    public double stdDevY;
    public double stdDevYaw;
    public boolean validUpdate;
    public String source;

    public PoseObservation(){
        pose = null;
        timestamp = 0;
        stdDevX = 0;
        stdDevY = 0;
        stdDevYaw = 0;
        validUpdate = false;
        source = "";
    }
}
