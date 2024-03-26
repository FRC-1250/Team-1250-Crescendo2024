// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {
  private int LINDEX = 22;
  private int RINDEX = 23;
  DigitalInput[] irArray = new DigitalInput[5];

  private final TalonFX rightindexer;
  private final TalonFX leftindexer;

  /** Creates a new indexer. */
  public indexer() {
    for (int i = 0; i < irArray.length; i++) {
      irArray[i] = new DigitalInput(i);
    }
        CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
    currentLimit.StatorCurrentLimitEnable = true;
    currentLimit.StatorCurrentLimit = 45;
    currentLimit.SupplyCurrentLimitEnable = true; 
    currentLimit.SupplyCurrentLimit = 25; 

    OpenLoopRampsConfigs openloopconfigs = new OpenLoopRampsConfigs();
    openloopconfigs.DutyCycleOpenLoopRampPeriod = .1;

    rightindexer = new TalonFX(RINDEX, "rio");
    rightindexer.getConfigurator().apply(currentLimit);
    rightindexer.getConfigurator().apply(openloopconfigs);

    leftindexer = new TalonFX(LINDEX, "rio");
    leftindexer.getConfigurator().apply(currentLimit);
    leftindexer.getConfigurator().apply(openloopconfigs);
  }

  public void setDutyoutIndex(double percent) {
    rightindexer.set(percent);
    leftindexer.set(percent);
  }

  boolean pollIrArraySensor(int index) {
    if (index < irArray.length && index > -1) {
      // The IR sensors being used return "true" when not tripped. Flip the result for
      // standard usage. i.e. when sensor is tripped, it returns true
      return !irArray[index].get();
    } else {
      return false;
    }
  }

  public void centernote() {
    boolean barrelsensor = pollIrArraySensor(0);
    boolean intakeside = pollIrArraySensor(1);
    boolean indexerclose = pollIrArraySensor(2);
    boolean indexerfar = pollIrArraySensor(3);
    boolean shooterside = pollIrArraySensor(4);
  

    if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false
        && barrelsensor == false) {
          //was .3
      setDutyoutIndex(.4);
    } else if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false
        && barrelsensor == true) {
      setDutyoutIndex(.05);
    } else if (intakeside == true && indexerclose == false && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.05); //all point ones after this were .05
    } else if (intakeside == true && indexerclose == true && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.05);
    } else if (intakeside == true && indexerclose == true && indexerfar == true && shooterside == false) {
      setDutyoutIndex(.05);
    } else if (intakeside == false && indexerclose == true && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.05);
    } else if (intakeside == false && indexerclose == false && indexerfar == true && shooterside == false) {
      setDutyoutIndex(-.05);
    } else if (intakeside == false && indexerclose == true && indexerfar == true && shooterside == false) {
      setDutyoutIndex(0);
    } else if (intakeside == false && indexerclose == true && indexerfar == true && shooterside == true) {
      setDutyoutIndex(-0.05);
    } else if (intakeside == false && indexerclose == false && indexerfar == true && shooterside == true) {
      setDutyoutIndex(-0.05);
    } else if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == true) {
      setDutyoutIndex(-0.2);
    }

  }

  public boolean iscentered() {
    boolean intakeside = pollIrArraySensor(1);
    boolean indexerclose = pollIrArraySensor(2);
    boolean indexerfar = pollIrArraySensor(3);
    boolean shooterside = pollIrArraySensor(4);

    return (intakeside == false && indexerclose == true && indexerfar == true && shooterside == false);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer/Right duty cycle", rightIndexSparkMax.get());
    SmartDashboard.putNumber("Indexer/Right stator current", rightIndexSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Indexer/Left duty cycle", leftIndexSparkmax.get());
    SmartDashboard.putNumber("Indexer/Left stator current", leftIndexSparkmax.getOutputCurrent());
    SmartDashboard.putBoolean("Indexer/Note centered", iscentered());
    SmartDashboard.putBoolean("Indexer/Barrel sensor", pollIrArraySensor(0));
  }
}
