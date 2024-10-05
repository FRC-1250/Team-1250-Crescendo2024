// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Launcher extends SubsystemBase {
  private final int LEFT_LAUNCHER_CAN_ID = 20;
  private final int RIGHT_LAUNCHER_CAN_ID = 21;

  //Rpms for different shots
  public final int FALCON_500_MAX_RPM = 6380;
  public final int PODIUM_TARGET_RPM = 5000;
  public final int SPEAKER_TARGET_RPM = 3500;
  public final int AMP_TARGET_RPM = 1500;

  private final VelocityVoltage leftVelocityControl;
  private final VelocityVoltage rightVelocityControl;
  private final TalonFX rightLauncher;
  private final TalonFX leftLauncher;
  private final TalonFXPerformanceMonitor rightLauncherMonitor;
  private final TalonFXPerformanceMonitor leftLauncherMonitor;

  /** Creates a new shooter. */

  public Launcher() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //Configurations for the shooter wheels for SPEAKER and PODIUMS shots 
    configs.Slot0.kP = 0.11;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0.0001;
    configs.Slot0.kV = 0.115;

    //The configurations for the shooter wheels when shooting podium shot
    configs.Slot1.kP = .11;
    configs.Slot1.kI = 0;
    configs.Slot1.kD = .0001;
    configs.Slot1.kV = .12;

    //Motor settings that are applied and given to the talons 
    configs.CurrentLimits.SupplyCurrentLimit = 50;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightLauncher = new TalonFX(RIGHT_LAUNCHER_CAN_ID, "rio");
    rightLauncher.getConfigurator().apply(configs);
    rightLauncherMonitor = new TalonFXPerformanceMonitor(rightLauncher, getSubsystem(), "RightLauncher");
    rightVelocityControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

    leftLauncher = new TalonFX(LEFT_LAUNCHER_CAN_ID, "rio");
    leftLauncher.getConfigurator().apply(configs);
    leftLauncherMonitor = new TalonFXPerformanceMonitor(leftLauncher, getSubsystem(), "LeftLauncher");
    leftVelocityControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    SmartDashboard.putNumber("Launcher/tuning RPM", 0);
  }

  public void SetDutyOutlaunch(double percent) {
    rightLauncher.set(percent);
    leftLauncher.set(percent);
  }

  public double getRightLauncherRPM() {
    return rightLauncher.getVelocity().getValue() * 60;
  }

  public double getLeftLauncherRPM() {
    return leftLauncher.getVelocity().getValue() * 60;
  }

  public void SetLauncherVelocity(double right, double left) {
    rightLauncher.setControl(rightVelocityControl.withVelocity(right / 60));
    leftLauncher.setControl(leftVelocityControl.withVelocity(left / 60));
  }

  public void telemeterize() {
    rightLauncherMonitor.telemeterize();
    leftLauncherMonitor.telemeterize();
  }

  @Override
  public void periodic() {

  }
}
