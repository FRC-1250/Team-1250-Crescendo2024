// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private final NetworkTable table;
  private long fID;
  private double tx, ty, tv, ta, ts = -1;
  private Double[] cameraPoseInTargetspace, botPoseWpiBlue, botPoseWpiRed;

  /**
   * Json dump processing
   */
  private ObjectMapper objectMapper = new ObjectMapper();
  private String json = "";

  public Limelight() {
    this("limelight");
  }

  public Limelight(String LimelightTable) {
    table = NetworkTableInstance.getDefault().getTable(LimelightTable);
  }

  /*
   * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   */
  public double getXOffset() {
    return tx;
  }

  /*
   * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   */
  public double getYOffset() {
    return ty;
  }

  /*
   * ta Target Area (0% of image to 100% of image)
   */
  public double getTargetAreaPercent() {
    return ta;
  }

  /*
   * ts The skew of the target
   */
  public double getSkew() {
    return ts;

  }

  public String getjsonString() {
    return json;
  }


  /*
   * Camera Pose in target space as computed by this fiducial (x,y,z,rx,ry,rz)
   */
  public Double[] getCameraPoseInTargetspace() {
    return cameraPoseInTargetspace;
  }

  /*
   * botpose_wpiblue - botpose, but with the origin at the right-hand side of the
   * driverstation on the blue side of the field.
   */
  public Double[] getBotPoseWpiBlue() {
    return botPoseWpiBlue;
  }

  /*
   * botPoseWpiRed - botpose, but with the origin at the right-hand side of the
   * driverstation on the red side of the field.
   */
  public Double[] getBotPoseWpiRed() {
    return botPoseWpiRed;
  }

  /*
   * Fiducial tag ID
   */
  public long getfid() {
    return fID;
  }

  /**
   * ledMode Sets limelight’s LED state
   * 0 use the LED Mode set in the current pipeline
   * 1 force off
   * 2 force blink
   * 3 force on
   */

  public void setLEDMode(int value) {
    if (value > 3 || value < 0)
      value = 0;
    table.getEntry("ledMode").setNumber(value);
  }

  public void setPipeline(int value) {
    if (value > 9 || value < 0)
      value = 0;
    table.getEntry("pipeline").setNumber(value);
  }

  public boolean isTargetSeen() {
    return tv == 1;
  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx").getDouble(-1);
    ty = table.getEntry("ty").getDouble(-1);
    tv = table.getEntry("tv").getDouble(-1);
    ts = table.getEntry("ts").getDouble(-1);
    ta = table.getEntry("ta").getDouble(-1);
    cameraPoseInTargetspace = table.getEntry("camerapose_targetspace").getDoubleArray(new Double[] {});
    botPoseWpiBlue = table.getEntry("botpose_wpiblue").getDoubleArray(new Double[] {});
    botPoseWpiRed = table.getEntry("botpose_wpired").getDoubleArray(new Double[] {});
    fID = table.getEntry("tid").getInteger(-1);
  }
}