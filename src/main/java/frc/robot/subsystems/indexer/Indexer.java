// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Indexer extends SubsystemBase {
  private final DigitalInput[] irArray = new DigitalInput[5];
  private final TalonFX indexer;
  private final DutyCycleOut indexerDutyCycleOut;
  private final VelocityVoltage indexerVelocityControl;
  private final TalonFXPerformanceMonitor indexerMonitor;

  /** Creates a new indexer. */
  public Indexer() {
    indexer = new TalonFX(IndexerConfig.CAN_ID, IndexerConfig.CAN_BUS);
    indexer.getConfigurator().apply(IndexerConfig.TALON_CONFIG);
    indexerDutyCycleOut = new DutyCycleOut(0);
    indexerVelocityControl = new VelocityVoltage(0);
    indexerMonitor = new TalonFXPerformanceMonitor(indexer, IndexerConfig.SUBSYTEM_NAME,
        IndexerConfig.INDEXER_STRING);

    for (int i = 0; i < irArray.length; i++) {
      irArray[i] = new DigitalInput(i);
    }
  }

  public Command setDutyCycle(double percentOut) {
    return Commands.runOnce(
        () -> {
          indexer.setControl(indexerDutyCycleOut.withOutput(percentOut));
        }, this).withName("SetIndexingRollersDutyCycle");
  }

  public Command setVelocity(double rotationsPerSecond) {
    return Commands.runOnce(
        () -> {
          indexer.setControl(indexerVelocityControl.withVelocity(rotationsPerSecond));
        }, this).withName("SetIndexingRollersVelocityVoltage");
  }

  public Command stageNote() {
    return new NotifierCommand(() -> {
      if (pollIrArraySensor(0)) {
        indexer.setControl(indexerDutyCycleOut.withOutput(0));
      } else {
        indexer.setControl(indexerDutyCycleOut.withOutput(IndexerConfig.STAGING_SPEED));
      }
    }, 0.01, this)
        .until(() -> isNoteStaged())
        .withName("StageNote");
  }

  public Command centerNote() {
    return new NotifierCommand(() -> {
      boolean bottom = pollIrArraySensor(1); // Indexer side
      boolean middleBottom = pollIrArraySensor(2);
      boolean middleTop = pollIrArraySensor(3);
      boolean top = pollIrArraySensor(4); // Shooter side

      if (middleBottom == true && bottom == true) {
        // stop
        indexer.setControl(indexerDutyCycleOut.withOutput(0));
      } else if ((top == true || middleTop == true) || (middleBottom == true && bottom == false)) {
        // Back
        indexer.setControl(indexerDutyCycleOut.withOutput(-IndexerConfig.CENTERING_SPEED));
      } else {
        // Forward
        indexer.setControl(indexerDutyCycleOut.withOutput(IndexerConfig.CENTERING_SPEED));
      }
    }, 0.01, this);
  }

  public boolean isNoteStaged() {
    return pollIrArraySensor(0);
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
    indexerMonitor.telemeterize();
  }
}
