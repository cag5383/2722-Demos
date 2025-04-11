package frc.robot.subsystems.Vision.Observations;

public class TagObservation {

    /*
     * This is a basic class used to group/package data returned by the limelight
     * when it detects a target.
     * Because of the complexity of the underlying data, it can be helpful to have
     * classes that we define on our own
     * to consolidate all of that data.
     * 
     * This class in particular is an example of trying to keep complicated problems
     * distinct from one another.
     * If we clearly specify what information we want from our vision solutions,
     * Other subsystems can ask for and use the neatly packaged data without
     * worrying about where it comes from.
     * Similarly, the vision system can do all manner of complex processing to
     * improve the quality of the data
     * without creating additional complexity for other subsystems.
     */

    public double targetX; // The X offset of the target relative to the camera (Left to Right)
    public double targetY; // The Y Offset of the target relative to the camera (Front to Back)
    public double targetYaw; // The angle of the target relative to the camera
                             // (measured clockwise from the front of the robot)
    public double ambiguity; // A measure of how confident the Limelight is in the measurement - the smaller
                             // the ambiguity, the higher the confidence.
    public int tagID; // The tag ID associated with the measurement. (e.g. this will be 7 if the measurements are for AprilTag 7)
    public boolean validUpdate; // an indication of whether this data has been updated or not.
    public String source; // Specifies the name of the limelight that produced the measurement.

    public TagObservation() {
        // CAG: Initialize to invalid.
        targetX = 0;
        targetY = 0;
        targetYaw = 0;
        ambiguity = 0;
        validUpdate = false;
        tagID = -1;
        source = "";
    }

}
