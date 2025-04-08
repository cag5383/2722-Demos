package frc.robot.subsystems.Vision;
public class VisionConstants {
    
    public static final double maxYawRate = 720; //degrees per second. CAG: I don't know what turn rate is too big to trust the measurements.
    public static final double maxAmbiguity = 0; //CAG: Maximum ambiguity for a vision measurement - I have no idea what a reasonable value is.
                                                    //Maybe we can find a good value from 2539?

    public static final class FieldConstants {
        public static final double fieldMinX = 0;
        public static final double fieldMaxX = 18; //CAG: meters - made up for now.
        public static final double fieldMinY = 0;
        public static final double fieldMaxY = 18; //CAG: made up for now.
    }
}