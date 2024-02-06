// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {

  public enum Position {
    LIMIT(100),
    AMP(80),
    PREP_CLIMB(60),
    CLIMB(40),
    SPEAKER(10),
    HOME(0);

    // Default value is rotations
    public final float value;

    Position(float value) {
      this.value = value;
    }
  }

  private final int LEFT_ROTATOR_CAN_ID = 30;
  private final int RIGHT_ROTATOR_CAN_ID = 31;
  private final int ALLOWABLE_CLOSED_LOOP_ERROR = 8192;

  // Offset value to normalize the encoder position to 0 when at home
  private final int ENCODER_OFFSET = 0;

  private final CANSparkMax leftRotator;
  private final CANSparkMax rightRotator;
  private final SparkPIDController rightRotatorPIDController;
  private final AbsoluteEncoder rightRotatorThroughBoreEncoder;
  private final RelativeEncoder rightRotatorInternalEncoder;

  public Shoulder() {
    rightRotator = new CANSparkMax(RIGHT_ROTATOR_CAN_ID, MotorType.kBrushless);
    rightRotator.restoreFactoryDefaults();
    rightRotator.setSmartCurrentLimit(60);
    rightRotator.setIdleMode(IdleMode.kBrake);
    rightRotator.setClosedLoopRampRate(0.5);
    rightRotator.setOpenLoopRampRate(0.5);
    rightRotator.setSoftLimit(SoftLimitDirection.kForward, Position.LIMIT.value);
    rightRotator.setSoftLimit(SoftLimitDirection.kReverse, Position.HOME.value);
    rightRotator.enableSoftLimit(SoftLimitDirection.kForward, false);
    rightRotator.enableSoftLimit(SoftLimitDirection.kReverse, false);
    rightRotator.setInverted(false);

    leftRotator = new CANSparkMax(LEFT_ROTATOR_CAN_ID, MotorType.kBrushless);
    leftRotator.restoreFactoryDefaults();
    leftRotator.setSmartCurrentLimit(60);
    leftRotator.setIdleMode(IdleMode.kBrake);
    leftRotator.setClosedLoopRampRate(0.5);
    leftRotator.setOpenLoopRampRate(0.5);
    leftRotator.follow(rightRotator, true);

    rightRotatorThroughBoreEncoder = rightRotator.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightRotatorThroughBoreEncoder.setInverted(false);
    rightRotatorThroughBoreEncoder.setZeroOffset(ENCODER_OFFSET);

    rightRotatorInternalEncoder = rightRotator.getEncoder();
    rightRotatorInternalEncoder.setInverted(false);

    rightRotatorPIDController = rightRotator.getPIDController();
    rightRotatorPIDController.setFeedbackDevice(rightRotatorInternalEncoder);
    rightRotatorPIDController.setP(0.1);
    rightRotatorPIDController.setI(0);
    rightRotatorPIDController.setD(1);
    rightRotatorPIDController.setFF(0);
    rightRotatorPIDController.setOutputRange(-1, 1);
  }

  public void setPosition(double targetPosition) {
    rightRotatorPIDController.setReference(targetPosition, ControlType.kPosition);
  }

  public void setDutyCycle(double percentOut) {
    rightRotator.set(percentOut);
  }

  public void stop() {
    rightRotator.set(0);
  }

  public double getPosition() {
    return rightRotatorThroughBoreEncoder.getPosition();
  }

  public void resetInternalEncoder() {
    rightRotatorInternalEncoder.setPosition(0);
  }

  public boolean isAtSetPoint(double targetPosition) {
    return Math.abs(targetPosition - getPosition()) < ALLOWABLE_CLOSED_LOOP_ERROR;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder position", rightRotatorInternalEncoder.getPosition());
  }
}
