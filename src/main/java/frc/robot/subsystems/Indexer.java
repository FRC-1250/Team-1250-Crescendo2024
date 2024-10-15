// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.TalonFXMonitor;
import frc.robot.telemetry.TelemetryManager;

public class Indexer extends SubsystemBase {
  private int LINDEX = 22;
  DigitalInput[] irArray = new DigitalInput[5];

  private final TalonFX leftindexer;

  /** Creates a new indexer. */
  public Indexer() {
    for (int i = 0; i < irArray.length; i++) {
      irArray[i] = new DigitalInput(i);
    }

    //Configurations/settings that are being set to the talonfx motor controller
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.CurrentLimits.SupplyCurrentLimitEnable = true; 
    configs.CurrentLimits.SupplyCurrentLimit = 20;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftindexer = new TalonFX(LINDEX, "rio");
    leftindexer.getConfigurator().apply(configs);
    TelemetryManager.getInstance().addTalonFX(new TalonFXMonitor(leftindexer, getSubsystem(), "LeftIndexer"));
  }

  public void setDutyoutIndex(double percent) {
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

  public void telemeterize() {
    SmartDashboard.putBoolean("Indexer/Note centered", iscentered());
    SmartDashboard.putBoolean("Indexer/Barrel sensor", pollIrArraySensor(0));
  }

  @Override
  public void periodic() {
    
  }
}
