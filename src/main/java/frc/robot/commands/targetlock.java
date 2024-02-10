// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class targetlock implements SwerveRequest {
  public double VelocityX = 0;
  public double VelocityY = 0;
  public double RotationalRate = 0;
  public double Deadband = 0;
  public double RotationalDeadband = 0;
  public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;
  protected SwerveModuleState[] m_lastAppliedState = null;
  public PhoenixPIDController headingController;

  private double kP = -0.15;
  private double KProt = 0.01;
  private double xCorrect = 0;
  private double yCorrect = 0;
  private double rotCorrect = 0;
  private double tx;
  private double ty;
  private Object drivetrain;
  private long fid;
  private Alliance alliance;
  private Limelight limelight;
  private double angle;
  /** Creates a new targetlock. */
  public targetlock() {
    Limelight limelight;
    int currentag;
    int lasttag;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    // TODO Auto-generated method 
        fid = limelight.getfid();
  if (alliance == Alliance.Blue) {
      if (fid == 1 || fid == 2) {
        angle = 135;
      } else if (fid == 6) {
        angle = -90;
      }
      else if (fid == 7 || fid == 8){
        angle = 0;
      }
    } else if (alliance == Alliance.Red) {
      if (fid == 3 || fid == 4) {
        angle = 180;
      } else if (fid == 5) {
        angle = 90;
      } else if (fid == 9 || fid == 10){
        angle = 135;
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
      
                  ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                          parameters.currentPose.getRotation()), parameters.updatePeriod);
      
                  var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());
      
                  for (int i = 0; i < modulesToApply.length; ++i) {
                      modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
                  }
    return StatusCode.OK;
}
}
