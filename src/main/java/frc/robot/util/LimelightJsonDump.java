package frc.robot.util;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightJsonDump {
    @JsonProperty("Results")
    private LimelightJsonDumpResults results = new LimelightJsonDumpResults();

    public LimelightJsonDumpResults getResults() {
        return results;
    }
}
