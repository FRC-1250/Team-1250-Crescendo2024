// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private final String name;

  public Limelight() {
    this("limelight");
  }

  public Limelight(String limelightTable) {
    name = limelightTable;
  }

  /**
   * Gets the Pose2d and timestamp for use with WPILib pose estimator
   * (addVisionMeasurement) when you are on the BLUE
   * alliance
   * 
   */
  public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2() {
    return getBotPoseEstimate("botpose_orb_wpiblue");
  }

  public void SetRobotOrientation(double yaw, double yawRate,
      double pitch, double pitchRate,
      double roll, double rollRate) {

    double[] entries = new double[6];
    entries[0] = yaw;
    entries[1] = yawRate;
    entries[2] = pitch;
    entries[3] = pitchRate;
    entries[4] = roll;
    entries[5] = rollRate;
    setLimelightNTDoubleArray(name, "robot_orientation_set", entries);
  }

  public double getTX() {
    return getLimelightNTDouble(name, "tx");
  }

  public double getTY() {
    return getLimelightNTDouble(name, "ty");
  }

  public double getTA() {
    return getLimelightNTDouble(name, "ta");
  }

  public double getFiducialID() {
    return getLimelightNTDouble(name, "tid");
  }

  public boolean getTV() {
    return 1.0 == getLimelightNTDouble(name, "tv");
  }

  public void setPipelineIndex(int pipelineIndex) {
    setLimelightNTDouble(name, "pipeline", pipelineIndex);
  }

  public void setPriorityTagID(int ID) {
    setLimelightNTDouble(name, "priorityid", ID);
  }

  /**
   * The LEDs will be controlled by Limelight pipeline settings, and not by robot
   * code.
   */

  public void setLEDMode_PipelineControl() {
    setLimelightNTDouble(name, "ledMode", 0);
  }

  public void setLEDMode_ForceOff() {
    setLimelightNTDouble(name, "ledMode", 1);
  }

  public void setLEDMode_ForceBlink() {
    setLimelightNTDouble(name, "ledMode", 2);
  }

  public void setLEDMode_ForceOn() {
    setLimelightNTDouble(name, "ledMode", 3);
  }

  @Override
  public void periodic() {
  }

  private double[] getLimelightNTDoubleArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  private String getLimelightNTString(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getString("");
  }

  private String[] getLimelightNTStringArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getStringArray(new String[0]);
  }

  private NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  private NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  private double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  private void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  private void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  private Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      // System.err.println("Bad LL 2D Pose Data!");
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  private double extractArrayEntry(double[] inData, int position) {
    if (inData.length < position + 1) {
      return 0;
    }
    return inData[position];
  }

  private PoseEstimate getBotPoseEstimate(String entryName) {
    var poseEntry = getLimelightNTTableEntry(name, entryName);
    var poseArray = poseEntry.getDoubleArray(new double[0]);
    var pose = toPose2D(poseArray);
    double latency = extractArrayEntry(poseArray, 6);
    int tagCount = (int) extractArrayEntry(poseArray, 7);
    double tagSpan = extractArrayEntry(poseArray, 8);
    double tagDist = extractArrayEntry(poseArray, 9);
    double tagArea = extractArrayEntry(poseArray, 10);
    // getlastchange() in microseconds, ll latency in milliseconds
    var timestamp = (poseEntry.getLastChange() / 1000000.0) - (latency / 1000.0);

    RawFiducial[] rawFiducials = new RawFiducial[tagCount];
    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.length != expectedTotalVals) {
      // Don't populate fiducials
    } else {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = (int) poseArray[baseIndex];
        double txnc = poseArray[baseIndex + 1];
        double tync = poseArray[baseIndex + 2];
        double ta = poseArray[baseIndex + 3];
        double distToCamera = poseArray[baseIndex + 4];
        double distToRobot = poseArray[baseIndex + 5];
        double ambiguity = poseArray[baseIndex + 6];
        rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
      }
    }
    return new PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
  }

  private String sanitizeName(String name) {
    if (name == "" || name == null) {
      return "limelight";
    }
    return name;
  }
}