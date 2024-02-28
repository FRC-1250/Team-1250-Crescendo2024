// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Limelight;

/** Add your docs here. */
public class FieldCentricAutoAim implements SwerveRequest {

    public double VelocityX = 0;
    public double VelocityY = 0;
    public double Deadband = 0;
    public double RotationalDeadband = 0;
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;
    private long fid;
    private Optional<Alliance> alliance;
    private final Limelight limelight;
    private double xOffset;
    private final double kP = 0.05;

    public FieldCentricAutoAim(Limelight limelight) {
        this.limelight = limelight;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        fid = limelight.getfid();
        alliance = DriverStation.getAlliance();

        if (fid == -1 || alliance.isEmpty()) {
            xOffset = 0;
        } else if (alliance.get() == Alliance.Blue) {
            if (fid == 7 || fid == 6) {
                xOffset = -limelight.getXOffset();
            } else {
                xOffset = 0;
            }
        } else if (alliance.get() == Alliance.Red) {
            if (fid == 4 || fid == 5) {
                xOffset = -limelight.getXOffset();
            } else {
                xOffset = 0;
            }
        }

        double toApplyX = VelocityX;
        double toApplyY = VelocityY;
        double toApplyOmega = xOffset * kP * TunerConstants.MaxAngularRate;

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

    public FieldCentricAutoAim withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    public FieldCentricAutoAim withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    public FieldCentricAutoAim withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    public FieldCentricAutoAim withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    public FieldCentricAutoAim withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    public FieldCentricAutoAim withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}
