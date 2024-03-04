// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;
import frc.robot.util.DistanceToAngle;

public class AutoShoulderAdjust extends Command {
  private final Shoulder shoulder;
  private final Limelight limelight;
  private final double SPEAKER_TAG_HEIGHT_METERS = Units.inchesToMeters(55);
  private double fid;
  private double calculatedDistance;
  private double calculatedAngle;
  private double calculatedRotations;

  public AutoShoulderAdjust(Limelight limelight, Shoulder shoulder) {
    addRequirements(shoulder);
    this.limelight = limelight;
    this.shoulder = shoulder;
  }

  @Override
  public void execute() {
    fid = limelight.getfid();
    if (fid == 4 || fid == 7) {
      calculatedDistance = limelight.calculateDistanceMeters(SPEAKER_TAG_HEIGHT_METERS);
      calculatedAngle = DistanceToAngle.distAngle(calculatedDistance);
      calculatedRotations = calculatedAngle / 360;
      shoulder.setPosition(calculatedRotations);
    }
  }
}
