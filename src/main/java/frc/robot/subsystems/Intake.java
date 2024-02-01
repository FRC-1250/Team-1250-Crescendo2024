// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    lowerRoller = new CANSparkMax(LOWER_ROLLER_CAN_ID, MotorType.kBrushless);
    lowerRoller.restoreFactoryDefaults();
    lowerRoller.setSmartCurrentLimit(40);

  }

  public void setDutyCycleUpperRoller(double percentOut) {
    upperRoller.set(percentOut);
  }

  public void setDutyCycleLowerRoller(double percentOut) {
    lowerRoller.set(percentOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
