// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launcher extends SubsystemBase {
  private int LSHOOTER = 20;
  private int RSHOOTER = 21;
  public final int maxRPM = 6380;
  public final int PODIUM_TARGET_RPM_LEFT = 5000;
  public final int PODIUM_TARGET_RPM_RIGHT = 5000;
  public final int SPEAKER_TARGET_RPM_LEFT = 4500;
  public final int SPEAKER_TARGET_RPM_RIGHT = 4500;
  public final int AMP_TARGET_RPM_LEFT = 1500;
  public final int AMP_TARGET_RPM_RIGHT = 1500; 
  private final VelocityVoltage velocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
private final TalonFX rightLauncher;
private final TalonFX leftLauncher;



  /** Creates a new shooter. */

  
  public launcher() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = .11;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0.0001;
    configs.Slot0.kV = .12;
    configs.CurrentLimits.StatorCurrentLimit = 45;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 45;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightLauncher = new TalonFX(RSHOOTER, "rio");
    rightLauncher.getConfigurator().apply(new TalonFXConfiguration());
    rightLauncher.getConfigurator().apply(configs);


    leftLauncher = new TalonFX(LSHOOTER, "rio");  
    leftLauncher.getConfigurator().apply(new TalonFXConfiguration());
    leftLauncher.getConfigurator().apply(configs);

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
    rightLauncher.setControl(velocity.withVelocity(right/60));
    leftLauncher.setControl(velocity.withVelocity(left/60));
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Right RPM", getRightLauncherRPM());
    SmartDashboard.putNumber("Launcher/Right stator current", rightLauncher.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Launcher/Left RPM", getLeftLauncherRPM());
    SmartDashboard.putNumber("Launcher/Left stator current", leftLauncher.getStatorCurrent().getValueAsDouble());
  }
}
