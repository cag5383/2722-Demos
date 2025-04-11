package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.Observations.TagObservation;

public class VisionAdapter extends SubsystemBase{
        private final Vision m_vision = new Vision();
        public TagObservation currTagObservation = new TagObservation();

        public VisionAdapter(){
            
        }

        public void periodic(){
            //periodic functions of subsystems are called once per robot cycle (about 50 times/second)
            //The vision adapter queries the vision system for the newest TargetObservation every robot cycle.
            //The entire purpose of this layer is to gather all of our vision information in one place in a consistent format
            //So that it is as easy as possible for other subsystems to consume.
            currTagObservation = m_vision.getTargetObservations();
        }

}
