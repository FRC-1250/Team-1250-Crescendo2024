// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetLock implements SwerveRequest {
  public double VelocityX = 0;
  public double VelocityY = 0;
  public double Deadband = 0;
  public double RotationalDeadband = 0;
  public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;
  public PhoenixPIDController headingController;
  private long fid;
  private Optional<Alliance> alliance;
  private final Limelight limelight;
  private double angle;

  /** Creates a new targetlock. */
  /**
   * Sets the velocity in the X direction, in m/s.
   * X is defined as forward according to WPILib convention,
   * so this determines how fast to travel forward.
   *
   * @param velocityX Velocity in the X direction, in m/s
   * @return this request
   */
  public TargetLock withVelocityX(double velocityX) {
    this.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s.
   * Y is defined as to the left according to WPILib convention,
   * so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  public TargetLock withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  public TargetLock withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }

  /**
   * Sets the type of control request to use for the drive motor.
   *
   * @param driveRequestType The type of control request to use for the drive
   *                         motor
   * @return this request
   */
  public TargetLock withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
    this.DriveRequestType = driveRequestType;
    return this;
  }

  /**
   * Sets the type of control request to use for the steer motor.
   *
   * @param steerRequestType The type of control request to use for the steer
   *                         motor
   * @return this request
   */
  public TargetLock withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }

  public TargetLock(Limelight limelight) {
    this.limelight = limelight;
    this.alliance = DriverStation.getAlliance();
  }

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    fid = limelight.getfid();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        if (fid == 1 || fid == 2) {
          angle = 135;
        } else if (fid == 6) {
          angle = -90;
        } else if (fid == 7 || fid == 8) {
          angle = 0;
        }
      } else if (alliance.get() == Alliance.Red) {
        if (fid == 3 || fid == 4) {
          angle = 180;
        } else if (fid == 5) {
          angle = 90;
        } else if (fid == 9 || fid == 10) {
          angle = 135;
        }
      }
    }

    double rotationRate = headingController.calculate(parameters.currentPose.getRotation().getRadians(),
        Math.toRadians(angle), parameters.timestamp);
    double toApplyX = VelocityX;
    double toApplyY = VelocityY;
    double toApplyOmega = rotationRate;
    if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
      toApplyX = 0;
      toApplyY = 0;
    }
    if (Math.abs(toApplyOmega) < RotationalDeadband) {
      toApplyOmega = 0;
    }

    ChassisSpeeds speeds = ChassisSpeeds
        .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
            parameters.currentPose.getRotation()), parameters.updatePeriod);

    var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
    }
    return StatusCode.OK;
  }
}
