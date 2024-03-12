// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {


  private final int FRONT_ROLLER_CAN_ID = 40;
  private final int REAR_ROLLER_CAN_ID = 41;
  private final TalonFX frontRoller;
  private final TalonFX rearRoller;

  public Intake() {
    CurrentLimitsConfigs currentlimits = new CurrentLimitsConfigs();
    currentlimits.StatorCurrentLimit = 40;
    currentlimits.StatorCurrentLimitEnable = true;

    OpenLoopRampsConfigs loopRampsConfigs = new OpenLoopRampsConfigs();
    loopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.1;

    frontRoller = new TalonFX(FRONT_ROLLER_CAN_ID, "rio");
    frontRoller.getConfigurator().apply(new TalonFXConfiguration());
    frontRoller.getConfigurator().apply(currentlimits);
    frontRoller.getConfigurator().apply(loopRampsConfigs);

    rearRoller = new TalonFX(REAR_ROLLER_CAN_ID, "rio");
    rearRoller.getConfigurator().apply(new TalonFXConfiguration());
    rearRoller.getConfigurator().apply(currentlimits);
    rearRoller.getConfigurator().apply(loopRampsConfigs);

  }

  public void setDutyCycleFrontRoller(double percentOut) {
    frontRoller.set(percentOut);
  }

  public void setDutyCycleRearRoller(double percentOut) {
    rearRoller.set(percentOut);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake front roller duty cycle", frontRoller.get());
    SmartDashboard.putNumber("Intake front roller stator current", frontRoller.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake rear roller duty cycle", rearRoller.get());
    SmartDashboard.putNumber("Intake rear roller stator current", rearRoller.getStatorCurrent().getValueAsDouble());
  }
}
