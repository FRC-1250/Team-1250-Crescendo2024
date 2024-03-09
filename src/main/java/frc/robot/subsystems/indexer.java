// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {
  private int LINDEX = 22;
  private int RINDEX = 23;
  DigitalInput[] irArray = new DigitalInput[5];
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
    rightIndexSparkMax.setOpenLoopRampRate(0.1);
    rightIndexSparkMax.setClosedLoopRampRate(0.1);

    leftIndexSparkmax.restoreFactoryDefaults();
    leftIndexSparkmax.follow(rightIndexSparkMax, true);
    leftIndexSparkmax.setIdleMode(IdleMode.kBrake);
    leftIndexSparkmax.setSmartCurrentLimit(40);
    leftIndexSparkmax.setOpenLoopRampRate(0.1);
    leftIndexSparkmax.setClosedLoopRampRate(0.1);

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
    boolean barrelsensor = pollIrArraySensor(0);
    boolean intakeside = pollIrArraySensor(1);
    boolean indexerclose = pollIrArraySensor(2);
    boolean indexerfar = pollIrArraySensor(3);
    boolean shooterside = pollIrArraySensor(4);
  

    if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false
        && barrelsensor == false) {
      setDutyoutIndex(0.3);
    } else if (intakeside == false && indexerclose == false && indexerfar == false && shooterside == false
        && barrelsensor == true) {
      setDutyoutIndex(.1);
    } else if (intakeside == true && indexerclose == false && indexerfar == false && shooterside == false) {
      setDutyoutIndex(.05);
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
      setDutyoutIndex(-0.5);
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
    SmartDashboard.putNumber("Right index duty cycle", leftIndexSparkmax.get());
    SmartDashboard.putNumber("Left index duty cycle", rightIndexSparkMax.get());
    SmartDashboard.putBoolean("Note centered", iscentered());
    SmartDashboard.putBoolean("Barrel sensor", pollIrArraySensor(0));
  }
}
