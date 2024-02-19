// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final int UPPER_ROLLER_CAN_ID = 40;
  private final int LOWER_ROLLER_CAN_ID = 41;
  private final CANSparkMax upperRoller;
  private final CANSparkMax lowerRoller;

  public Intake() {
    upperRoller = new CANSparkMax(UPPER_ROLLER_CAN_ID, MotorType.kBrushless);
    upperRoller.restoreFactoryDefaults();
    upperRoller.setSmartCurrentLimit(40);
    upperRoller.setIdleMode(IdleMode.kCoast);
    upperRoller.setOpenLoopRampRate(0.1);

    lowerRoller = new CANSparkMax(LOWER_ROLLER_CAN_ID, MotorType.kBrushless);
    lowerRoller.restoreFactoryDefaults();
    lowerRoller.setSmartCurrentLimit(40);
    lowerRoller.setIdleMode(IdleMode.kCoast);
    lowerRoller.setOpenLoopRampRate(0.1);
  }

  public void setDutyCycleUpperRoller(double percentOut) {
    upperRoller.set(percentOut);
  }

  public void setDutyCycleLowerRoller(double percentOut) {
    lowerRoller.set(percentOut);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake upper duty cycle", upperRoller.get());
    SmartDashboard.putNumber("Intake lower duty cycle", lowerRoller.get());
  }
}
