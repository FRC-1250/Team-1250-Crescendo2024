// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {
  private int LINDEX = 22;
  private int RINDEX = 23;
  DigitalInput[] irArray = new DigitalInput[4];
  CANSparkMax leftIndexSparkmax = new CANSparkMax(LINDEX, MotorType.kBrushless);
  CANSparkMax rightIndexSparkMax = new CANSparkMax(RINDEX, MotorType.kBrushless);

  /** Creates a new indexer. */
  public indexer() {
    for (int i = 0; i < irArray.length; i++) {
      irArray[i] = new DigitalInput(i);
    }
    rightIndexSparkMax.restoreFactoryDefaults();
    rightIndexSparkMax.setIdleMode(IdleMode.kBrake);
    rightIndexSparkMax.setInverted(true);
    rightIndexSparkMax.setSmartCurrentLimit(40);

    leftIndexSparkmax.restoreFactoryDefaults();
    leftIndexSparkmax.follow(rightIndexSparkMax, true);
    leftIndexSparkmax.setIdleMode(IdleMode.kBrake);
    leftIndexSparkmax.setSmartCurrentLimit(40);

  }

  public void setDutyoutIndex(double percent) {
    rightIndexSparkMax.set(percent);
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
    boolean intakeside = pollIrArraySensor(0);
    boolean indexerclose = pollIrArraySensor(1);
    boolean indexerfar = pollIrArraySensor(2);
    boolean shooterside = pollIrArraySensor(3);

    if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false) {
      setDutyoutIndex(0.3);
    } else if (intakeside == true && indexerclose == false && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.1);
    } else if (intakeside == true && indexerclose == true && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.1);
    } else if (intakeside == true && indexerclose == true && indexerfar == true && shooterside == false) {
      setDutyoutIndex(.1);
     } else if (intakeside == false && indexerclose == true && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.1);
    } else if (intakeside == false && indexerclose == false && indexerfar == true && shooterside == false) {
      setDutyoutIndex(-.1);
    }else if (intakeside == false && indexerclose == true && indexerfar == true && shooterside == false) {
      setDutyoutIndex(0);
    } else if (intakeside == false && indexerclose == true && indexerfar == true && shooterside == true) {
      setDutyoutIndex(-0.1);
    } else if (intakeside == false && indexerclose == false && indexerfar == true && shooterside == true) {
      setDutyoutIndex(-0.1);
    } else if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == true) {
      setDutyoutIndex(-0.1);
    }

    
  }
public boolean iscentered() {
     boolean intakeside = pollIrArraySensor(0);
    boolean indexerclose = pollIrArraySensor(1);
    boolean indexerfar = pollIrArraySensor(2);
    boolean shooterside = pollIrArraySensor(3);

    return (intakeside == false && indexerclose == true && indexerfar == true && shooterside == false);
     
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
