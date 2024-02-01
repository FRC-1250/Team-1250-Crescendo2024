// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {
  
  CANSparkMax leftIndexSparkmax = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkMax rightIndexSparkMax = new CANSparkMax(23, MotorType.kBrushless);
  /** Creates a new indexer. */
  public indexer() {
     rightIndexSparkMax.restoreFactoryDefaults();
    rightIndexSparkMax.setIdleMode(IdleMode.kCoast);
    rightIndexSparkMax.setInverted(true);
    rightIndexSparkMax.setSmartCurrentLimit(40);

    leftIndexSparkmax.restoreFactoryDefaults();
    leftIndexSparkmax.follow(rightIndexSparkMax, true);
    leftIndexSparkmax.setIdleMode(IdleMode.kCoast);
    leftIndexSparkmax.setSmartCurrentLimit(40);



  }

  public void setDutyoutIndex(double percent){
    rightIndexSparkMax.set(percent);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
