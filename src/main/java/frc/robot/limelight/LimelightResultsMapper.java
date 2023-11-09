package frc.robot.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightResultsMapper {
    @JsonProperty("Results")
    public LimelightResults targetingResults;

    public LimelightResultsMapper() {
        targetingResults = new LimelightResults();
    }
}