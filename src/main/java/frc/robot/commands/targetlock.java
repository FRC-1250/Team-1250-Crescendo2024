// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.Limelight;

public class Targetlock implements SwerveRequest {
  public double VelocityX = 0;
  public double VelocityY = 0;
  public double Deadband = 0;
  public double RotationalDeadband = 0;
  public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;
  public PhoenixPIDController headingController = new PhoenixPIDController(4, 0, 0);
  private double fid;
  private Optional<Alliance> alliance;
  private final Limelight limelight;
  private final Supplier<Double> headingSupplier;
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
  public Targetlock withVelocityX(double velocityX) {
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
  public Targetlock withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  public Targetlock withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }

  /**
   * Sets the rotational deadband of the request.
   *
   * @param rotationalDeadband Rotational deadband of the request
   * @return this request
   */
  public Targetlock withRotationalDeadband(double rotationalDeadband) {
    this.RotationalDeadband = rotationalDeadband;
    return this;
  }

  /**
   * Sets the type of control request to use for the drive motor.
   *
   * @param driveRequestType The type of control request to use for the drive
   *                         motor
   * @return this request
   */
  public Targetlock withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
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
  public Targetlock withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }

  public Targetlock(Limelight limelight, Supplier<Double> headingSupplier) {
    this.limelight = limelight;
    this.headingSupplier = headingSupplier;
  }

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    fid = limelight.getFiducialID();
    if (fid == -1) {
      angle = headingSupplier.get();
    } else {
      alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        switch (alliance.get()) {
          case Blue:
            if (fid == 6) {
              angle = -90; // AMP
            } else if (fid == 7 || fid == 8) {
              angle = 0; // Speaker
            } else if (fid == 1 || fid == 2) {
              angle = 120; // Source
            }
            break;
          case Red:
            if (fid == 5) {
              angle = 90; // AMP
            } else if (fid == 3 || fid == 4) {
              angle = 0; // Speaker
            } else if (fid == 9 || fid == 10) {
              angle = -120; // Source
            }
            break;
          default:
            break;
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
