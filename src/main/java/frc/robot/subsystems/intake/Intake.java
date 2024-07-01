// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Intake extends SubsystemBase {
  private final TalonFX frontRoller;
  private final TalonFX rearRoller;
  private final TalonFXPerformanceMonitor frontRollerMonitor;
  private final TalonFXPerformanceMonitor rearRollerMonitor;

  public Intake() {
    frontRoller = new TalonFX(IntakeConfig.FRONT_ROLLER_CAN_ID);
    frontRoller.getConfigurator().apply(IntakeConfig.TALON_CONFIG);
    frontRollerMonitor = new TalonFXPerformanceMonitor(frontRoller, IntakeConfig.SUBSYTEM_NAME, IntakeConfig.FRONT_ROLLER_STRING);

    rearRoller = new TalonFX(IntakeConfig.REAR_ROLLER_CAN_ID);
    rearRoller.getConfigurator().apply(IntakeConfig.TALON_CONFIG);
    rearRollerMonitor = new TalonFXPerformanceMonitor(rearRoller, IntakeConfig.SUBSYTEM_NAME, IntakeConfig.REAR_ROLLER_STRING);
  }

  public Command setDutyCycle(double percentOut) {
    return Commands.runOnce(() -> {
      frontRoller.set(percentOut);
      rearRoller.set(percentOut);
    }, this);
  }

  @Override
  public void periodic() {
    frontRollerMonitor.telemeterize();
    rearRollerMonitor.telemeterize();
  }
}
