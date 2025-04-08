package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Observations.PoseObservation;
import frc.robot.subsystems.Vision.Observations.TagObservation;

public class Vision extends SubsystemBase {
    private List<Limelight> limelights;

    // CAG: unclear if we should be using MT2 STDDevs when we leverage the pose
    // observations or not.
    // 2539 calculates STDDevs based on average tag distance.
    private List<PoseObservation> poseObservations;

    // CAG: here's a fun question - what do we do with multiple target observations?
    // MT2 disambiguates by adding multiple poses with different stddevs - I don't
    // know what to do hear.
    private List<TagObservation> tagObservations;

    public Vision() {
        limelights = new ArrayList<>();
        limelights.add(new Limelight("l_limelight"));
        limelights.add(new Limelight("r_limelight"));

        poseObservations = new ArrayList<>();
        tagObservations = new ArrayList<>();


        //each limelight should have its own slot in the pose/tag lists
        for (int i = 0; i < limelights.size(); i++) {
            //Add empty/invalid observations to initialize the lists.
            poseObservations.add(new PoseObservation());
            tagObservations.add(new TagObservation());
        }
    }

    public void setPose(Pose2d pose) {
        // pass the pose to each limelight/vision system.
        // For limelights, this is useful for MegaTag.
        for (Limelight l : limelights) {
            l.setPose(pose);
        }
    }

    public void setYawRate(double yawRate) {
        // CAG:I think there's a cleaner way to do this with a state tracker or
        // suppliers.
        for (Limelight l : limelights) {
            l.setYawRate(yawRate);
        }
    }

    @Override
    public void periodic(){

        //For each limelight:
        for (int i = 0; i < limelights.size(); i++) {
            Limelight l = limelights.get(i);
            l.update(); //update the pose/tag estiamtes

            //CAG: we could have different data structures for the vision system and limelight system levels.
            //Vision system can just pass through whatever the limelight has.
            //Or we could do averaging/validating.
            //I'm going to do the easy version for now.
            poseObservations.set(i, l.currMegaTagObservation);
            tagObservations.set(i,l.currLockObservation);
        }
    }

    public void setLocks(int tagID){
        //CAG: I'm thinking a lock can be triggered by controller inputs and passed here by a state machine kind of deal.
        for (Limelight l : limelights) {
            l.lock(tagID);
        }
    }
}
