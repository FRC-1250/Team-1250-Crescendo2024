// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Launcher extends SubsystemBase {
  private final DigitalInput[] irArray = new DigitalInput[5];
  private final TalonFX rightLauncher;
  private final TalonFX leftLauncher;
  private final TalonFX indexer;
  private final VelocityVoltage leftVelocityControl;
  private final VelocityVoltage rightVelocityControl;
  private final TalonFXPerformanceMonitor rightLauncherMonitor;
  private final TalonFXPerformanceMonitor leftLauncherMonitor;
  private final TalonFXPerformanceMonitor indexerMonitor;

  public Launcher() {
    rightLauncher = new TalonFX(LauncherConfig.RIGHT_LAUNCHER_CAN_ID, LauncherConfig.CAN_BUS);
    rightLauncher.getConfigurator().apply(LauncherConfig.LAUNCHER_TALON_CONFIG);
    rightLauncherMonitor = new TalonFXPerformanceMonitor(rightLauncher, LauncherConfig.SUBSYTEM_NAME,
        LauncherConfig.RIGHT_LAUNCHER_STRING);
    rightVelocityControl = new VelocityVoltage(0);

    leftLauncher = new TalonFX(LauncherConfig.LEFT_LAUNCHER_CAN_ID, LauncherConfig.CAN_BUS);
    leftLauncher.getConfigurator().apply(LauncherConfig.LAUNCHER_TALON_CONFIG);
    leftLauncherMonitor = new TalonFXPerformanceMonitor(leftLauncher, LauncherConfig.SUBSYTEM_NAME,
        LauncherConfig.LEFT_LAUNCHER_STRING);
    leftVelocityControl = new VelocityVoltage(0);

    indexer = new TalonFX(LauncherConfig.INDEXER_CAN_ID, LauncherConfig.CAN_BUS);
    indexer.getConfigurator().apply(LauncherConfig.INDEXER_TALON_CONFIG);
    indexerMonitor = new TalonFXPerformanceMonitor(indexer, LauncherConfig.SUBSYTEM_NAME,
        LauncherConfig.INDEXER_STRING);

    for (int i = 0; i < irArray.length; i++) {
      irArray[i] = new DigitalInput(i);
    }
  }

  public void setDutyCycleLauncher(double percent) {
    rightLauncher.set(percent);
    leftLauncher.set(percent);
  }

  public double getRightLauncherRPM() {
    return rightLauncher.getVelocity().getValue() * 60;
  }

  public double getLeftLauncherRPM() {
    return leftLauncher.getVelocity().getValue() * 60;
  }

  public void setLauncherVelocity(double value) {
    rightLauncher.setControl(rightVelocityControl.withVelocity(value / 60));
    leftLauncher.setControl(leftVelocityControl.withVelocity(value / 60));
  }

  public void setDutyCycleIndexer(double percent) {
    indexer.set(percent);
  }

  public void centerNote() {
    boolean barrelsensor = pollIrArraySensor(0);
    boolean intakeside = pollIrArraySensor(1);
    boolean indexerclose = pollIrArraySensor(2);
    boolean indexerfar = pollIrArraySensor(3);
    boolean shooterside = pollIrArraySensor(4);

    if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false
        && barrelsensor == false) {
      // was .3
      setDutyCycleIndexer(.4);
    } else if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false
        && barrelsensor == true) {
      setDutyCycleIndexer(.05);
    } else if (intakeside == true && indexerclose == false && indexerfar == false && shooterside == false) {
      setDutyCycleIndexer(.05); // all point ones after this were .05
    } else if (intakeside == true && indexerclose == true && indexerfar == false && shooterside == false) {
      setDutyCycleIndexer(.05);
    } else if (intakeside == true && indexerclose == true && indexerfar == true && shooterside == false) {
      setDutyCycleIndexer(.05);
    } else if (intakeside == false && indexerclose == true && indexerfar == false && shooterside == false) {
      setDutyCycleIndexer(.05);
    } else if (intakeside == false && indexerclose == false && indexerfar == true && shooterside == false) {
      setDutyCycleIndexer(-.05);
    } else if (intakeside == false && indexerclose == true && indexerfar == true && shooterside == false) {
      setDutyCycleIndexer(0);
    } else if (intakeside == false && indexerclose == true && indexerfar == true && shooterside == true) {
      setDutyCycleIndexer(-0.05);
    } else if (intakeside == false && indexerclose == false && indexerfar == true && shooterside == true) {
      setDutyCycleIndexer(-0.05);
    } else if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == true) {
      setDutyCycleIndexer(-0.2);
    }
  }

  public boolean hasNote() {
    return pollIrArraySensor(0);
  }

  public boolean isNoteCentered() {
    boolean intakeside = pollIrArraySensor(1);
    boolean indexerclose = pollIrArraySensor(2);
    boolean indexerfar = pollIrArraySensor(3);
    boolean shooterside = pollIrArraySensor(4);

    return (intakeside == false && indexerclose == true && indexerfar == true && shooterside == false);
  }

  private boolean pollIrArraySensor(int index) {
    if (index < irArray.length && index > -1) {
      // The IR sensors being used return "true" when not tripped. Flip the result for
      // standard usage. i.e. when sensor is tripped, it returns true
      return !irArray[index].get();
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    leftLauncherMonitor.telemeterize();
    rightLauncherMonitor.telemeterize();
    indexerMonitor.telemeterize();
  }
}
