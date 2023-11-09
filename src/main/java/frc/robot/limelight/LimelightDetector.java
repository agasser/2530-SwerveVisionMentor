package frc.robot.limelight;

import java.lang.reflect.Array;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightDetector {

    /** Human-readable class name string */
    @JsonProperty("class")
    public String humanReadableClass;

    /** ClassID integer */
    @JsonProperty("classID")
    public double classID;

        /** Confidence of the predicition */
    @JsonProperty("conf")
    public double predicitionConfidence;

    /** Individual corner points as an array of {x,y} in pixels. Center-zero, positive right and down. Must be enabled. */
    @JsonProperty("pts")
    public Array[] individualCornerPoints;

    /** The size of the target as a percentage of the image (0-1) */
    @JsonProperty("ta")
    public double targetArea;

    /** X-coordinate of the center of the target in degrees. Positive-right, center-zero */
    @JsonProperty("tx")
    public double targetXDegrees;

    /** X-coordinate of the center of the target in pixels. Positive-right, center-zero */
    @JsonProperty("txp")
    public double targetXPixels;

    /** Y-coordinate of the center of the target in degrees. Positive-down, center-zero */
    @JsonProperty("ty")
    public double targetYDegrees;

    /** Y-coordinate of the center of the target in pixels. Positive-down, center-zero */
    @JsonProperty("typ")
    public double targetYPixels;

    public LimelightDetector() {
    }
    
}