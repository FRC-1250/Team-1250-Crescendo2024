package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonIgnoreProperties(ignoreUnknown = true)
public class LimelightJsonDumpResults {
    @JsonProperty("Fiducial")
    private List<ProcessedAprilTag> aprilTags = new ArrayList<>();

    @JsonProperty("tl")
    private double targetingLatencyInMs = -1; // milliseconds consumed by tracking loop this frame

    @JsonProperty("ts")
    private double timestampInMs = -1;

    @JsonProperty("pID")
    private int currentPipelineIndex = -1;

    @JsonProperty("v")
    private int validityIndicator = -1; // 1 = valid targets, 0 = no valid targets

    public List<ProcessedAprilTag> getAprilTags() {
        return aprilTags;
    }

    public double getTargetingLatencyInMs() {
        return targetingLatencyInMs;
    }

    public double getTimestampInMs() {
        return timestampInMs;
    }

    public int getCurrentPipelineIndex() {
        return currentPipelineIndex;
    }

    public int getValidityIndicator() {
        return validityIndicator;
    }
}
