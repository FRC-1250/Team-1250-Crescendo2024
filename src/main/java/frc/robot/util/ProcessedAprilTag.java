package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonIgnoreProperties(ignoreUnknown = true)
public class ProcessedAprilTag {
    @JsonProperty("fID")
    private int fiducialTagID = -1;

    @JsonProperty("fam")
    private String fiducialFamily = "";

    @JsonProperty("t6c_ts")
    private List<Double> cameraPoseInTargetSpace = new ArrayList<>();

    @JsonProperty("t6r_fs")
    private List<Double> robotPoseInFieldSpace = new ArrayList<>();

    @JsonProperty("t6r_ts")
    private List<Double> robotPoseInTargetSpace = new ArrayList<>();

    @JsonProperty("t6t_cs")
    private List<Double> targetPoseInCameraSpace = new ArrayList<>();

    @JsonProperty("t6t_rs")
    private List<Double> targetPoseInRobotSpace = new ArrayList<>();

    @JsonProperty("ta")
    private double ta = 0.0;

    @JsonProperty("tx")
    private double tx = 0.0;

    @JsonProperty("txp")
    private double txp = 0.0;

    @JsonProperty("ty")
    private double ty = 0.0;

    @JsonProperty("typ")
    private double typ = 0.0;

    public int getFiducialTagID() {
        return fiducialTagID;
    }

    public String getFiducialFamily() {
        return fiducialFamily;
    }

    public List<Double> getCameraPoseInTargetSpace() {
        return cameraPoseInTargetSpace;
    }

    public List<Double> getRobotPoseInFieldSpace() {
        return robotPoseInFieldSpace;
    }

    public List<Double> getRobotPoseInTargetSpace() {
        return robotPoseInTargetSpace;
    }

    public List<Double> getTargetPoseInCameraSpace() {
        return targetPoseInCameraSpace;
    }

    public List<Double> getTargetPoseInRobotSpace() {
        return targetPoseInRobotSpace;
    }

    public double getTa() {
        return ta;
    }

    public double getTx() {
        return tx;
    }

    public double getTxp() {
        return txp;
    }

    public double getTy() {
        return ty;
    }

    public double getTyp() {
        return typ;
    }
}

