// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderV1 extends SubsystemBase {
  public enum Position {
    AMP(.358f),
    HORIZONTAL(.25f),
    SPEAKER_PODIUM(.158f),
    SPEAKER(0.1059f),
    HOME(.055f),
    PID(.194f);

    public final float value; // Default value is rotations

    Position(float value) {
      this.value = value;
    }
  }

  private final int LEFT_ROTATOR_CAN_ID = 30;
  private final int RIGHT_ROTATOR_CAN_ID = 31;
  private final double CLOSED_LOOP_TOLERANCE = 0.003; // Closed loop tolerance in degrees
  private final double ENCODER_OFFSET = 0.21; // Offset value to normalize the encoder position to 0 when at home

  private final CANSparkMax leftRotator;
  private final CANSparkMax rightRotator;
  private final SparkPIDController rightRotatorPIDController;
  private final AbsoluteEncoder rightRotatorThroughBoreEncoder;

  public ShoulderV1() {
    rightRotator = new CANSparkMax(RIGHT_ROTATOR_CAN_ID, MotorType.kBrushless);
    rightRotator.restoreFactoryDefaults();
    rightRotator.setSmartCurrentLimit(25);
    rightRotator.setIdleMode(IdleMode.kBrake);
    rightRotator.setSoftLimit(SoftLimitDirection.kForward, Position.AMP.value);
    rightRotator.setSoftLimit(SoftLimitDirection.kReverse, Position.HOME.value);
    rightRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);
    rightRotator.setInverted(false);

    leftRotator = new CANSparkMax(LEFT_ROTATOR_CAN_ID, MotorType.kBrushless);
    leftRotator.restoreFactoryDefaults();
    leftRotator.setSmartCurrentLimit(25);
    leftRotator.setIdleMode(IdleMode.kBrake);
    leftRotator.follow(rightRotator, true);

    rightRotatorThroughBoreEncoder = rightRotator.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightRotatorThroughBoreEncoder.setInverted(true);
    rightRotatorThroughBoreEncoder.setZeroOffset(ENCODER_OFFSET);

    rightRotatorPIDController = rightRotator.getPIDController();
    rightRotatorPIDController.setFeedbackDevice(rightRotatorThroughBoreEncoder);
    rightRotatorPIDController.setP(25);
    rightRotatorPIDController.setI(0);
    rightRotatorPIDController.setD(0);
    rightRotatorPIDController.setFF(0);
    rightRotatorPIDController.setOutputRange(-1, 1);
    SmartDashboard.putNumber("Launcher/tuning pos", Position.HOME.value);
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

  public boolean isAtSetPoint(double targetPosition) {
    return MathUtil.isNear(targetPosition, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  public boolean isAtHome() {
    return MathUtil.isNear(Position.HOME.value, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  public boolean isNearSetPoint(double targetPosition) {
    return MathUtil.isNear(targetPosition, getPosition(), CLOSED_LOOP_TOLERANCE * 7.5);
    //if we modify the scale value, we need to double check shooter positions to check for overlap till the bounce is fixed
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder/Absoulte position", rightRotatorThroughBoreEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder/Degress position", rightRotatorThroughBoreEncoder.getPosition() * 360);
    SmartDashboard.putNumber("Shoulder/Right duty cycle", rightRotator.get());
    SmartDashboard.putNumber("Shoulder/Left duty cycle", leftRotator.get());
    SmartDashboard.putNumber("Shoulder/Right stator current", rightRotator.getOutputCurrent());
    SmartDashboard.putNumber("Shoulder/Left stator current", leftRotator.getOutputCurrent());
  }
}