// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Intake extends SubsystemBase {

  private final int FRONT_ROLLER_CAN_ID = 40;
  private final int REAR_ROLLER_CAN_ID = 41;
  private final TalonFX frontRoller;
  private final TalonFX rearRoller;
  private final TalonFXPerformanceMonitor frontRollerMonitor;
  private final TalonFXPerformanceMonitor rearRollerMonitor;

  public Intake() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    frontRoller = new TalonFX(FRONT_ROLLER_CAN_ID, "rio");
    frontRoller.getConfigurator().apply(talonFXConfiguration);
    frontRollerMonitor = new TalonFXPerformanceMonitor(frontRoller, getSubsystem(), "FrontRoller");

    rearRoller = new TalonFX(REAR_ROLLER_CAN_ID, "rio");
    rearRoller.getConfigurator().apply(talonFXConfiguration);
    rearRollerMonitor = new TalonFXPerformanceMonitor(rearRoller, getSubsystem(), "RearRoller");
  }

  public void setDutyCycleFrontRoller(double percentOut) {
    frontRoller.set(percentOut);
  }

  public void setDutyCycleRearRoller(double percentOut) {
    rearRoller.set(percentOut);
  }

  public void telemeterize() {
    frontRollerMonitor.telemeterize();
    rearRollerMonitor.telemeterize();
  }

  @Override
  public void periodic() {
   
  }
}
