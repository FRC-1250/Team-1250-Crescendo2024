// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launcher extends SubsystemBase {
  private int LSHOOTER = 20;
  private int RSHOOTER = 21;
  public final int maxRPM = 5700;
  public final int PODIUM_TARGET_RPM_LEFT = 5000;
  public final int PODIUM_TARGET_RPM_RIGHT = 5000;
  public final int SPEAKER_TARGET_RPM_LEFT = 4500;
  public final int SPEAKER_TARGET_RPM_RIGHT = 4500;
  public final int AMP_TARGET_RPM_LEFT = 1500;
  public final int AMP_TARGET_RPM_RIGHT = 1500; 

private final TalonFX rightLauncher;
private final TalonFX leftLauncher;



  /** Creates a new shooter. */

  
  public launcher() {
    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
    currentLimit.StatorCurrentLimitEnable = true;
    currentLimit.StatorCurrentLimit = 45;
    currentLimit.SupplyCurrentLimitEnable = true; 
    currentLimit.SupplyCurrentLimit = 25; 

    OpenLoopRampsConfigs openloopconfigs = new OpenLoopRampsConfigs();
    openloopconfigs.DutyCycleOpenLoopRampPeriod = .1;

    rightLauncher = new TalonFX(RSHOOTER, "rio");
    rightLauncher.getConfigurator().apply(new TalonFXConfiguration());
    rightLauncher.getConfigurator().apply(currentLimit);
    rightLauncher.getConfigurator().apply(openloopconfigs);


    leftLauncher = new TalonFX(LSHOOTER, "rio");  
    leftLauncher.getConfigurator().apply(currentLimit);
    leftLauncher.getConfigurator().apply(openloopconfigs);

  }

public void SetDutyOutlaunch(double percent) {
  rightLauncher.set(percent);
  leftLauncher.set(percent);
}

public double getRightLauncherRPM() {
 return rightLauncher.getVelocity().getValue();
}

public double getLeftLauncherRPM() {
 return leftLauncher.getVelocity().getValue();
}

public void SetLauncherVelocity(double right, double left) {
    rightLauncherPIDController.setReference(right, ControlType.kVelocity);
    leftLauncerPIDController.setReference(left, ControlType.kVelocity);
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Right RPM", getRightLauncherRPM());
    SmartDashboard.putNumber("Launcher/Right stator current", rightLauncherSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Launcher/Left RPM", getLeftLauncherRPM());
    SmartDashboard.putNumber("Launcher/Left stator current", leftLauncherSparkMax.getOutputCurrent());
  }
}
