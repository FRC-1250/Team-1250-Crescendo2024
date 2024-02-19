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

public class Shoulder extends SubsystemBase {

  public enum Position {
    LIMIT(.347f),
    AMP(.33f),
    HORIZONTAL(.25f),
    SPEAKER_PODIUM(.111f),
    SPEAKER(.083f),
    HOME(.068f),
    PID(.194f);

    // Default value is rotations
    public final float value;

    Position(float value) {
      this.value = value;
    }
  }

  private final int LEFT_ROTATOR_CAN_ID = 30;
  private final int RIGHT_ROTATOR_CAN_ID = 31;

  // Closed loop tolerance in degrees
  // TODO: Use isAtSetPoint after determining new home of ABS sensor and changing rotations to degrees
  private final double CLOSED_LOOP_TOLERANCE = 0.02;

  // Offset value to normalize the encoder position to 0 when at home
  private final double ENCODER_OFFSET = 0.134;

  private final CANSparkMax leftRotator;
  private final CANSparkMax rightRotator;
  private final SparkPIDController rightRotatorPIDController;
  private final AbsoluteEncoder rightRotatorThroughBoreEncoder;

  public Shoulder() {
    rightRotator = new CANSparkMax(RIGHT_ROTATOR_CAN_ID, MotorType.kBrushless);
    rightRotator.restoreFactoryDefaults();
    rightRotator.setSmartCurrentLimit(25);
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
    leftRotator.setSmartCurrentLimit(25);
    leftRotator.setIdleMode(IdleMode.kBrake);
    leftRotator.setClosedLoopRampRate(0.5);
    leftRotator.setOpenLoopRampRate(0.5);
    leftRotator.follow(rightRotator, true);

    rightRotatorThroughBoreEncoder = rightRotator.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightRotatorThroughBoreEncoder.setInverted(true);
    //rightRotatorThroughBoreEncoder.setPositionConversionFactor(ENCODER_POSITION_CONVERSION_FACTOR);
    rightRotatorThroughBoreEncoder.setZeroOffset(ENCODER_OFFSET);

    rightRotatorPIDController = rightRotator.getPIDController();
    rightRotatorPIDController.setFeedbackDevice(rightRotatorThroughBoreEncoder);
    rightRotatorPIDController.setP(10);
    rightRotatorPIDController.setI(0);
    rightRotatorPIDController.setD(0);
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

  public boolean isAtSetPoint(double targetPosition) {
    return MathUtil.isNear(targetPosition, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Shoulder position, abs", rightRotatorThroughBoreEncoder.getPosition());
    SmartDashboard.putNumber("Shoulder position, abs degrees", rightRotatorThroughBoreEncoder.getPosition() * 360);
    SmartDashboard.putNumber("Shoulder velocity, abs", rightRotatorThroughBoreEncoder.getVelocity());
  }
}
