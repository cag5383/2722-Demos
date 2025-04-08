package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Vision.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.Vision.Observations.PoseObservation;
import frc.robot.subsystems.Vision.Observations.TagObservation;
import frc.robot.subsystems.Vision.VisionConstants.FieldConstants;

public class Limelight {
    private String ll_name; // the name of the limelight as configured in the webapp
    private int lock_id; // the april tag ID that we're currently locked on.
    private boolean doRejectUpdate; // is there a valid MT2 update this frame.
    // Orientation data for Megatag - passed in periodically(RobotContainer ->
    // Vision -> here)
    private Pose2d currPose;
    private double currYawRate;

    private LimelightHelpers.PoseEstimate currpPoseEstimate;
    public PoseObservation currMegaTagObservation;
    public TagObservation currLockObservation;

    // MegaTag StdDevs
    private double MT1X;
    private double MT1Y;
    private double MT1Z;
    private double MT1Roll;
    private double MT1Pitch;
    private double MT1Yaw;
    private double MT2X;
    private double MT2Y;
    private double MT2Z;
    private double MT2Roll;
    private double MT2Pitch;
    private double MT2Yaw;

    private boolean rejectLockUpdate;

    public Limelight(String name) {
        ll_name = name;
        lock_id = -1; // CAG: set this to -1 when not locked, and to the tag ID when locked.
        // CAG: setCameraPose_RobotSpace - maybe better to do this in code than webapp.
    }

    public void update() {

        // CAG: we could do this in a periodic - but the order that periodics happen in
        // is going to start to matter.
        // i.e. does the Limelight update before or after the thing that's asking it for
        // updates?
        // It may be the case that - eventually - we want some kind of global periodic
        // that drives the periodic updates in a particular order.
        updateMT2(); // Get MT2 data and stddevs.
        toggleLEDMode();

        // if system is locked onto a tag id
        if (lock_id > 0) {
            updateLockData();
        }
    }

    private void updateMT2() {
        // CAG: this sets the robot orientation for MegaTag2 - it uses the current
        // orientation to solve for
        // where the robot is.
        LimelightHelpers.SetRobotOrientation(ll_name, currPose.getRotation().getDegrees(),
                currYawRate, 0, 0, 0, 0); // CAG: the rest of these are hopefully zero unless something has gone wrong.
        // Get MT2 pose
        currpPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll_name);
        // Query the network tables for MegaTag standard deviations
        updateMegaTagStdDevs(); // CAG: Very curious if these STDDEVs change over time or if they are
                                // configurable values.
                                // 2539 builds a linear relationship between average tag distance and STDDevs.

        // CAG: Would love to see the vision updates in a method and clearly annotated.
        // We should also think about how we want pose estimation/vision to work.
        // Do we want a pose estimation subsystem that combines all the measurements
        // rather than
        // The drivetrain getting more and more complex every time we add more vision
        // inputs?
        // Do we want a vision subsystem that queries all the cameras and processes all
        // the inputs?
        // or are we comfortable with that living in the pose estimator/drivetrain?
        // If we want other commands/subsystems to be able to be aware of pose info
        // without
        // Needing to actually drive the subsystem, pulling that information out may
        // make sense.
        doRejectUpdate = currpPoseEstimate == null || // CAG: If the data is bad (usually because limelight is still
                                                      // booting)
                currpPoseEstimate.tagCount == 0 || // if there are no tags visible
                Math.abs(currYawRate) > VisionConstants.maxYawRate ||
                // mt2 doesn't provide ambiguity - unclear what's happening to make that not
                // problematic.
                (currpPoseEstimate.pose.getX() < FieldConstants.fieldMinX
                        || currpPoseEstimate.pose.getX() > FieldConstants.fieldMaxX)
                || // PoseX
                   // is in
                   // bounds.
                (currpPoseEstimate.pose.getY() < FieldConstants.fieldMinY
                        || currpPoseEstimate.pose.getY() > FieldConstants.fieldMaxY);
        // CAG: are we stressed about Z? we do our best to ignore it the rest of the
        // time.

        // CAG: initialize to invalid and then populate.
        // If the update is good - populate the current MegaTag Observation
        PoseObservation thisUpdate = new PoseObservation();

        if (!doRejectUpdate) {

            thisUpdate.pose = currpPoseEstimate.pose;
            thisUpdate.timestamp = currpPoseEstimate.timestampSeconds;
            thisUpdate.stdDevX = MT2X;
            thisUpdate.stdDevY = MT2Y;
            thisUpdate.stdDevYaw = Double.POSITIVE_INFINITY; // CAG: Positive infinity is because
                                                             // we trust the gyro/pidgeon more than
                                                             // MT2 for orientation.
            thisUpdate.source = ll_name;
        }
        this.currMegaTagObservation = thisUpdate;
    }

    public void lock(int tag_id) {
        lock_id = tag_id;
    }

    public void unlock() {
        // clear the data
        lock_id = -1;
        currLockObservation = null;
    }

    public void setPose(Pose2d pose) {
        currPose = pose;
    }

    public void setYawRate(double yawRate) {
        currYawRate = yawRate;
    }

    private void updateMegaTagStdDevs() {
        double[] stddevs = NetworkTableInstance.getDefault().getTable(ll_name).getEntry("stddevs")
                .getDoubleArray(new double[12]);

        MT1X = stddevs[0];
        MT1Y = stddevs[1];
        MT1Z = stddevs[2];
        MT1Roll = stddevs[3];
        MT1Pitch = stddevs[4];
        MT1Yaw = stddevs[5];
        MT2X = stddevs[6];
        MT2Y = stddevs[7];
        MT2Z = stddevs[8];
        MT2Roll = stddevs[9];
        MT2Pitch = stddevs[10];
        MT2Yaw = stddevs[11];
    }

    private void toggleLEDMode() {
        // CAG: I'm not sure this is necessary, but want to see if it works/helps.
        // Idea is to start blinking the LED if we see a tag.

        if (currMegaTagObservation == null || currpPoseEstimate.tagCount < 1) { // if no data or no tags
            NetworkTableInstance.getDefault().getTable(ll_name).getEntry("ledMode").setNumber(1); // Set to Force Off
        } else { // otherwise, there must be a tag. Therefore:
            NetworkTableInstance.getDefault().getTable(ll_name).getEntry("ledMode").setNumber(2); // Set to Force Blink
        }
    }

    private void updateLockData() {

        // CAG: the target fiducial results have a bunch of data
        // specified in the Limelight Helpers class definition.
        // What it doesn't provide is the ambiguity
        // So if we want to gather all of that, we have to work for it.

        // I'm assuming these two sets of information are correlated - which may or may
        // not be true.
        // One quick sanity check would be to see whether they are always the same
        // length.
        // We could try to organize all of the data and so something smart with it -
        // like
        // treating each targetFiducial pose independently instead of letting MegaTag do
        // them.
        // But for now - just search each one for the the lock ID.

        LimelightTarget_Fiducial[] targetFiducials = LimelightHelpers.getLatestResults(ll_name).targets_Fiducials;

        Pose2d lockPose = null;
        double lockAmbiguity = -1;

        for (RawFiducial f : currpPoseEstimate.rawFiducials) {
            if (f.id == lock_id) {
                lockAmbiguity = f.ambiguity;
                break;
            }
        }

        for (LimelightTarget_Fiducial f : targetFiducials) {
            if (f.fiducialID == lock_id) {
                lockPose = f.getTargetPose_RobotSpace2D();
                break;
            }
        }

        rejectLockUpdate = (lockPose == null || lockAmbiguity == -1 || lockAmbiguity > VisionConstants.maxAmbiguity);
        // if the ambiguity is less than some maximum ambiguity - no idea what this
        // should be.

        // CAG: initialize to invalid and then populate if it's not.
        TagObservation thisUpdate = new TagObservation();
        if (!rejectLockUpdate) {
            thisUpdate.targetX = lockPose.getX();
            thisUpdate.targetY = lockPose.getY();
            thisUpdate.targetYaw = lockPose.getRotation().getDegrees();
            thisUpdate.ambiguity = lockAmbiguity;
            thisUpdate.tagID = lock_id;
            thisUpdate.validUpdate = true;
            thisUpdate.source = ll_name;
        }

        currLockObservation = thisUpdate;
    }
}
