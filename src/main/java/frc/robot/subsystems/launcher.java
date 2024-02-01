// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launcher extends SubsystemBase {
  private int LSHOOTER = 20;
  private int RSHOOTER = 21;
  private int LINDEX = 22;
  private int RINDEX = 23;

  private final int maxRPM = 5700;
  /** Creates a new shooter. */
  CANSparkMax leftLauncherSparkMax = new CANSparkMax(20, MotorType.kBrushless);
  CANSparkMax rightLauncherSparkMax = new CANSparkMax(21, MotorType.kBrushless);
  DigitalInput[] irArray = new DigitalInput[6];
  SparkPIDController pidController;

  public launcher() {
   
    rightLauncherSparkMax.restoreFactoryDefaults();
    rightLauncherSparkMax.setIdleMode(IdleMode.kCoast);
    rightLauncherSparkMax.setInverted(true);
    rightLauncherSparkMax.setSmartCurrentLimit(40);

    leftLauncherSparkMax.restoreFactoryDefaults();
    leftLauncherSparkMax.setIdleMode(IdleMode.kCoast);
    leftLauncherSparkMax.follow(rightLauncherSparkMax, true);
    leftLauncherSparkMax.setSmartCurrentLimit(40);

    pidController = rightLauncherSparkMax.getPIDController();

    pidController.setP(.1);
    pidController.setI(.1);
    pidController.setD(0);
    pidController.setFF(0.1);
    
    

    for (int i = 0; i < irArray.length; i++) {
      irArray[i] = new DigitalInput(i);
    }
  }


  boolean pollIrArraySensor(int index) {
    if (index < irArray.length && index > -1) {
      return irArray[index].get();
    } else {
      return false;
    }
  }




public void SetDutyOutlaunch(double percent) {
  rightLauncherSparkMax.set(percent);
}

public void SetLauncherVelocity(double setpoint) {
    pidController.setReference(setpoint, ControlType.kVelocity);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
