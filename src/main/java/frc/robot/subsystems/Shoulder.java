// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
    AMP(.358f),
    HORIZONTAL(.25f),
    SPEAKER_PODIUM(.158f),
    SPEAKER(.105f),
    HOME(.055f),
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
  private final double CLOSED_LOOP_TOLERANCE = 0.003;

  // Offset value to normalize the encoder position to 0 when at home
  private final double ENCODER_OFFSET = 0.134;

  private final TalonFX leftRotator;
  private final TalonFX rightRotator;
  private final SparkPIDController rightRotatorPIDController;
  private final AbsoluteEncoder rightRotatorThroughBoreEncoder;

  public Shoulder() {
    CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
    currentLimit.StatorCurrentLimitEnable = true;
    currentLimit.StatorCurrentLimit = 35;
    currentLimit.SupplyCurrentLimitEnable = true; 
    currentLimit.SupplyCurrentLimit = 25; 

    OpenLoopRampsConfigs openloopconfigs = new OpenLoopRampsConfigs();
    openloopconfigs.DutyCycleOpenLoopRampPeriod = .1;


    rightRotator = new TalonFX(RIGHT_ROTATOR_CAN_ID, "rio");
    rightRotator.getConfigurator().apply(new TalonFXConfiguration());
    rightRotator.getConfigurator().apply(currentLimit);
    rightRotator.getConfigurator().apply(openloopconfigs);


    leftRotator = new TalonFX(LEFT_ROTATOR_CAN_ID, "rio");
    leftRotator.getConfigurator().apply(new TalonFXConfiguration());
    leftRotator.getConfigurator().apply(currentLimit);
    leftRotator.getConfigurator().apply(openloopconfigs);


    rightRotatorThroughBoreEncoder = rightRotator.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    rightRotatorThroughBoreEncoder.setInverted(true);
    //rightRotatorThroughBoreEncoder.setPositionConversionFactor(ENCODER_POSITION_CONVERSION_FACTOR);
    rightRotatorThroughBoreEncoder.setZeroOffset(ENCODER_OFFSET);

    



    //rightRotatorPIDController = rightRotator.set;
    rightRotatorPIDController.setFeedbackDevice(rightRotatorThroughBoreEncoder);
    rightRotatorPIDController.setP(25);
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

  public boolean isAtHome() {
    return MathUtil.isNear(Position.HOME.value, getPosition(), CLOSED_LOOP_TOLERANCE);
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
