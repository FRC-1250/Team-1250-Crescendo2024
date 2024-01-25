// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {

  public enum ShoulderRotation {
    LIMIT(100),
    AMP(80),
    PREP_CLIMB(60),
    CLIMB(40),
    SPEAKER(10),
    HOME(0);

    public final double positionInTicks;

    ShoulderRotation(double positionInTicks) {
      this.positionInTicks = positionInTicks;
    }
  }

  private final int LEFT_CAN_ID = 11;
  private final int RIGHT_CAN_ID = 12;
  private final CANSparkMax left;
  private final CANSparkMax right;
  private final SparkPIDController sparkPIDController;
  private final RelativeEncoder throughBoreEncoder;

  public Shoulder() {
    right = new CANSparkMax(RIGHT_CAN_ID, MotorType.kBrushless);
    right.restoreFactoryDefaults();
    right.setSmartCurrentLimit(60);
    right.setIdleMode(IdleMode.kBrake);
    right.setClosedLoopRampRate(0.5);
    right.setOpenLoopRampRate(0.5);

    left = new CANSparkMax(LEFT_CAN_ID, MotorType.kBrushless);
    left.restoreFactoryDefaults();
    left.setSmartCurrentLimit(60);
    left.setIdleMode(IdleMode.kBrake);
    left.setClosedLoopRampRate(0.5);
    left.setOpenLoopRampRate(0.5);
    left.follow(right, true);

    throughBoreEncoder = right.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    sparkPIDController = right.getPIDController();
    sparkPIDController.setFeedbackDevice(throughBoreEncoder);
    sparkPIDController.setP(0.1);
    sparkPIDController.setI(0);
    sparkPIDController.setD(1);
    sparkPIDController.setOutputRange(-1, 1);
  }

  public void setPosition(double rotations) {
    sparkPIDController.setReference(rotations, ControlType.kPosition);
  }

  public void setDutyCycle(double percentOut) {
    right.set(percentOut);
  }

  public void stop() {
    right.set(0);
  }

  public double getPosition() {
    return throughBoreEncoder.getPosition();
  }

  public void resetPosition() {
    throughBoreEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
  }
}
