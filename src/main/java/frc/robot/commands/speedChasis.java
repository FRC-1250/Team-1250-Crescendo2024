// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TunerConstants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;

public class speedChasis extends Command {
  /** Creates a new speedChasis. */
  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();


  private final ChassisSpeeds chassisSpeeds;
  public double xMetersPerSecond;
  public double yMetersPerSecond;
  public double omegaRadiansPerSecond;

  public speedChasis(double xMetersPerSecond, double yMetersPerSecond, double omegaRadiansPerSecond) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xMetersPerSecond = xMetersPerSecond;
    this.yMetersPerSecond = yMetersPerSecond;
    this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    chassisSpeeds = new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TunerConstants.DriveTrain.setControl(autoRequest.withSpeeds(chassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
