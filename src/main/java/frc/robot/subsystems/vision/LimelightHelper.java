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

public class LimelightHelper {

  /**
   * Gets the Pose2d and timestamp for use with WPILib pose estimator
   * (addVisionMeasurement) when you are on the BLUE
   * alliance
   * 
   */
  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String tableName) {
    return getBotPoseEstimate(tableName, "botpose_orb_wpiblue");
  }

  public static void SetRobotOrientation(String tableName, double yaw, double yawRate,
      double pitch, double pitchRate,
      double roll, double rollRate) {

    double[] entries = new double[6];
    entries[0] = yaw;
    entries[1] = yawRate;
    entries[2] = pitch;
    entries[3] = pitchRate;
    entries[4] = roll;
    entries[5] = rollRate;
    setLimelightNTDoubleArray(tableName, "robot_orientation_set", entries);
  }

  public static double getTX(String tableName) {
    return getLimelightNTDouble(tableName, "tx");
  }

  public static double getTY(String tableName) {
    return getLimelightNTDouble(tableName, "ty");
  }

  public static double getTA(String tableName) {
    return getLimelightNTDouble(tableName, "ta");
  }

  public static double getFiducialID(String tableName) {
    return getLimelightNTDouble(tableName, "tid");
  }

  public static boolean getTV(String tableName) {
    return 1.0 == getLimelightNTDouble(tableName, "tv");
  }

  public static void setPipelineIndex(String tableName, int pipelineIndex) {
    setLimelightNTDouble(tableName, "pipeline", pipelineIndex);
  }

  public static void setPriorityTagID(String tableName,int ID) {
    setLimelightNTDouble(tableName, "priorityid", ID);
  }

  /**
   * The LEDs will be controlled by Limelight pipeline settings, and not by robot
   * code.
   */

  public static void setLEDMode_PipelineControl(String tableName) {
    setLimelightNTDouble(tableName, "ledMode", 0);
  }

  public static void setLEDMode_ForceOff(String tableName) {
    setLimelightNTDouble(tableName, "ledMode", 1);
  }

  public static void setLEDMode_ForceBlink(String tableName) {
    setLimelightNTDouble(tableName, "ledMode", 2);
  }

  public static void setLEDMode_ForceOn(String tableName) {
    setLimelightNTDouble(tableName, "ledMode", 3);
  }

  private static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  private static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  private static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  private static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  private static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  private static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      // System.err.println("Bad LL 2D Pose Data!");
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  private static double extractArrayEntry(double[] inData, int position) {
    if (inData.length < position + 1) {
      return 0;
    }
    return inData[position];
  }

  private static PoseEstimate getBotPoseEstimate(String tableName, String entryName) {
    var poseEntry = getLimelightNTTableEntry(tableName, entryName);
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

  private static String sanitizeName(String tableName) {
    if (tableName == "" || tableName == null) {
      return "limelight";
    }
    return tableName;
  }
}