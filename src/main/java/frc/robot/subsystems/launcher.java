// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launcher extends SubsystemBase {
  /** Creates a new shooter. */
   CANSparkMax leftLauncherSparkMax = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax rightLauncherSparkMax =  new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax leftIndexSparkmax = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax rightIndexSparkMax = new CANSparkMax(0, MotorType.kBrushless);
  public launcher() {
    rightIndexSparkMax.setIdleMode(IdleMode.kCoast);
    rightIndexSparkMax.setInverted(true);
    rightIndexSparkMax.setSmartCurrentLimit(40);
    leftIndexSparkmax.follow(rightIndexSparkMax, true);
    leftIndexSparkmax.setIdleMode(IdleMode.kCoast);

    rightLauncherSparkMax.setIdleMode(IdleMode.kCoast);
    rightLauncherSparkMax.setInverted(true);
    rightLauncherSparkMax.setSmartCurrentLimit(40);
    leftLauncherSparkMax.setIdleMode(IdleMode.kCoast);
    leftLauncherSparkMax.follow(rightLauncherSparkMax, true);
    
    
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
