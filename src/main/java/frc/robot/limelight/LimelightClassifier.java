package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightClassifier {

    /** Human-readable class name string */
    @JsonProperty("class")
    public double humanReadableClass;

    /** ClassID integer */
    @JsonProperty("classID")
    public double classID;

        /** Confidence of the predicition */
    @JsonProperty("conf")
    public double predicitionConfidence;

    public LimelightClassifier() {
    }
    
}