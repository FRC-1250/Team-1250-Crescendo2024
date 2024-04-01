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

public class launcher extends SubsystemBase {
  private final int LEFT_LAUNCHER_CAN_ID = 20;
  private final int RIGHT_LAUNCHER_CAN_ID = 21;
  public final int FALCON_500_MAX_RPM = 6380;
  public final int PODIUM_TARGET_RPM_LEFT = 5000;
  public final int PODIUM_TARGET_RPM_RIGHT = 5000;
  public final int SPEAKER_TARGET_RPM_LEFT = 4500;
  public final int SPEAKER_TARGET_RPM_RIGHT = 4500;
  public final int AMP_TARGET_RPM_LEFT = 1500;
  public final int AMP_TARGET_RPM_RIGHT = 1500;

  private final VelocityVoltage leftVelocityControl;
  private final VelocityVoltage rightVelocityControl;
  private final TalonFX rightLauncher;
  private final TalonFX leftLauncher;

  /** Creates a new shooter. */

  public launcher() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.Slot0.kP = 0.11;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0.0001;
    configs.Slot0.kV = 0.12;
    configs.CurrentLimits.SupplyCurrentLimit = 50;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightLauncher = new TalonFX(RIGHT_LAUNCHER_CAN_ID, "rio");
    rightLauncher.getConfigurator().apply(configs);
    rightVelocityControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

    leftLauncher = new TalonFX(LEFT_LAUNCHER_CAN_ID, "rio");
    leftLauncher.getConfigurator().apply(configs);
    leftVelocityControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Right RPM", getRightLauncherRPM());
    SmartDashboard.putNumber("Launcher/Right stator current", rightLauncher.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Launcher/Left RPM", getLeftLauncherRPM());
    SmartDashboard.putNumber("Launcher/Left stator current", leftLauncher.getStatorCurrent().getValueAsDouble());
  }
}
