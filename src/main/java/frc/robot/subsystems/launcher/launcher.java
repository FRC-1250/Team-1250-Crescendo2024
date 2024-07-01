// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Launcher extends SubsystemBase {
  private final TalonFX rightLauncher;
  private final TalonFX leftLauncher;
  private final DutyCycleOut leftDutyCycleOut;
  private final DutyCycleOut rightDutyCycleOut;
  private final VelocityVoltage leftVelocityControl;
  private final VelocityVoltage rightVelocityControl;
  private final Supplier<Double> leftVelocitySupplier;
  private final Supplier<Double> rightVelocitySupplier;
  private final TalonFXPerformanceMonitor rightLauncherMonitor;
  private final TalonFXPerformanceMonitor leftLauncherMonitor;

  public Launcher() {
    rightLauncher = new TalonFX(LauncherConfig.RIGHT_LAUNCHER_CAN_ID, LauncherConfig.CAN_BUS);
    rightLauncher.getConfigurator().apply(LauncherConfig.LAUNCHER_TALON_CONFIG);
    rightLauncherMonitor = new TalonFXPerformanceMonitor(rightLauncher, LauncherConfig.SUBSYTEM_NAME,
        LauncherConfig.RIGHT_LAUNCHER_STRING);
    rightVelocityControl = new VelocityVoltage(0);
    rightDutyCycleOut = new DutyCycleOut(0);
    rightVelocitySupplier = rightLauncher.getVelocity().asSupplier();

    leftLauncher = new TalonFX(LauncherConfig.LEFT_LAUNCHER_CAN_ID, LauncherConfig.CAN_BUS);
    leftLauncher.getConfigurator().apply(LauncherConfig.LAUNCHER_TALON_CONFIG);
    leftLauncherMonitor = new TalonFXPerformanceMonitor(leftLauncher, LauncherConfig.SUBSYTEM_NAME,
        LauncherConfig.LEFT_LAUNCHER_STRING);
    leftVelocityControl = new VelocityVoltage(0);
    leftDutyCycleOut = new DutyCycleOut(0);
    leftVelocitySupplier = leftLauncher.getVelocity().asSupplier();
  }

  public Command setDutyCycle(double percentOut) {
    return Commands.run(
        () -> {
          rightLauncher.setControl(rightDutyCycleOut.withOutput(percentOut));
          leftLauncher.setControl(leftDutyCycleOut.withOutput(percentOut));
        }, this).withName("SetShootingRollersDutyCycle");
  }

  public Command setVelocityVoltage(double rotationsPerSecond) {
    return Commands.runOnce(
        () -> {
          rightLauncher.setControl(rightVelocityControl.withVelocity(rotationsPerSecond));
          leftLauncher.setControl(leftVelocityControl.withVelocity(rotationsPerSecond));
        }, this)
        .withName("SetVelocity: " + rotationsPerSecond);
  }

  public Command setVelocityVoltage(ELauncherSpeed launcherSpeed) {
    return setVelocityVoltage(launcherSpeed.value / 60);
  }

  public Command waitUntilAtSetpoint() {
    return Commands.waitUntil(() -> isAtSetPoint()).withTimeout(LauncherConfig.VELOCITY_CLOSED_LOOP_TIME_OVERRIDE);
  }

  @Override
  public void periodic() {
    leftLauncherMonitor.telemeterize();
    rightLauncherMonitor.telemeterize();
  }

  private double getRightLauncherRotationsPerSecond() {
    return rightVelocitySupplier.get();
  }

  private double getLeftLauncherRotationsPerSecond() {
    return leftVelocitySupplier.get();
  }

  private boolean isAtSetPoint() {
    return MathUtil.isNear(rightLauncher.getClosedLoopReference().getValueAsDouble(),
        getRightLauncherRotationsPerSecond(),
        LauncherConfig.VELOCITY_CLOSED_LOOP_TOLERNACE)
        && MathUtil.isNear(leftLauncher.getClosedLoopReference().getValueAsDouble(),
            getLeftLauncherRotationsPerSecond(),
            LauncherConfig.VELOCITY_CLOSED_LOOP_TOLERNACE);
  }
}
