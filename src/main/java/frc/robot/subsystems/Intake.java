// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.TalonFXMonitor;
import frc.robot.telemetry.TelemetryManager;

public class Intake extends SubsystemBase {

  private final int FRONT_ROLLER_CAN_ID = 40;
  private final int REAR_ROLLER_CAN_ID = 41;
  private final TalonFX frontRoller;
  private final TalonFX rearRoller;

  public Intake() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    frontRoller = new TalonFX(FRONT_ROLLER_CAN_ID, "rio");
    frontRoller.getConfigurator().apply(talonFXConfiguration);
    rearRoller = new TalonFX(REAR_ROLLER_CAN_ID, "rio");
    rearRoller.getConfigurator().apply(talonFXConfiguration);
    TelemetryManager.getInstance().addTalonFX(new TalonFXMonitor(frontRoller, getSubsystem(), "FrontRoller"));
    TelemetryManager.getInstance().addTalonFX(new TalonFXMonitor(rearRoller, getSubsystem(), "RearRoller"));
  }

  public void setDutyCycleFrontRoller(double percentOut) {
    frontRoller.set(percentOut);
  }

  public void setDutyCycleRearRoller(double percentOut) {
    rearRoller.set(percentOut);
  }

  @Override
  public void periodic() {
   
  }
}
