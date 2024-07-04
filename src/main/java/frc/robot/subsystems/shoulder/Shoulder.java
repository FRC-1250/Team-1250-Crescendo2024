// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoulder;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANCoderPerformanceMonitor;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Shoulder extends SubsystemBase {
  private final TalonFX leftRotator;
  private final TalonFX rightRotator;
  private final CANcoder cancoder;
  private final DutyCycleOut dutyCycleOut;
  private final MotionMagicDutyCycle motionMagicDutyCycle;
  private final Supplier<Double> positionSupplier;
  private final CANCoderPerformanceMonitor canCoderPerformanceMonitor;
  private final TalonFXPerformanceMonitor rightRotatorMonitor;
  private final TalonFXPerformanceMonitor leftRotatorMonitor;

  public Shoulder() {
    cancoder = new CANcoder(ShoulderConfig.CANCODER_CAN_ID, ShoulderConfig.CAN_BUS);
    cancoder.getConfigurator().apply(ShoulderConfig.CANCODER_CONFIGS);
    positionSupplier = cancoder.getPosition().asSupplier();
    canCoderPerformanceMonitor = new CANCoderPerformanceMonitor(cancoder, ShoulderConfig.SUBSYSTEM_NAME,
        ShoulderConfig.CANCODER_STRING);
    BaseStatusSignal.setUpdateFrequencyForAll(200, cancoder.getPosition(), cancoder.getVelocity());

    rightRotator = new TalonFX(ShoulderConfig.RIGHT_ROTATOR_CAN_ID, ShoulderConfig.CAN_BUS);
    rightRotator.getConfigurator().apply(ShoulderConfig.TALON_FX_CONFIGURATION);
    rightRotatorMonitor = new TalonFXPerformanceMonitor(rightRotator, ShoulderConfig.SUBSYSTEM_NAME,
        ShoulderConfig.RIGHT_ROTATOR_STRING);

    leftRotator = new TalonFX(ShoulderConfig.LEFT_ROTATOR_CAN_ID, ShoulderConfig.CAN_BUS);
    leftRotator.getConfigurator().apply(ShoulderConfig.TALON_FX_CONFIGURATION);
    leftRotator.setControl(new Follower(rightRotator.getDeviceID(), true));
    leftRotatorMonitor = new TalonFXPerformanceMonitor(leftRotator, ShoulderConfig.SUBSYSTEM_NAME,
        ShoulderConfig.LEFT_ROTATOR_STRING);

    dutyCycleOut = new DutyCycleOut(0);
    motionMagicDutyCycle = new MotionMagicDutyCycle(0);
  }

  @Override
  public void periodic() {
    rightRotatorMonitor.telemeterize();
    leftRotatorMonitor.telemeterize();
    canCoderPerformanceMonitor.telemeterize();
  }

  public Command setMotionMagicDutyCycle(double rotations) {
    return Commands.runOnce(
        () -> {
          rightRotator.setControl(motionMagicDutyCycle.withPosition(rotations)
              .withFeedForward(0.1)
              .withLimitForwardMotion(isForwardLimit())
              .withLimitReverseMotion(isReverseLimit()));
        }, this)
        .withName("SetPosition: " + rotations);
  }

  public Command setMotionMagicDutyCycle(EShoulerPosition position) {
    return setMotionMagicDutyCycle(position.value);
  }

  public Command setMotionMagicAndWait(EShoulerPosition position) {
    return Commands.sequence(setMotionMagicDutyCycle(position),
        Commands.waitSeconds(0.75)).withName("SetPosition: " + position.toString());
  }

  public Command setDutyCycle(double percentOut) {
    return Commands.run(
        () -> {
          rightRotator.setControl(dutyCycleOut.withOutput(percentOut)
              .withLimitForwardMotion(isForwardLimit())
              .withLimitReverseMotion(isReverseLimit()));
        }, this).withName("SetDutyCycle");
  }

  public boolean isAtPoint(EShoulerPosition position) {
    return MathUtil.isNear(position.value, getPosition(), ShoulderConfig.CLOSED_LOOP_TOLERANCE);
  }

  public boolean isNearPoint(EShoulerPosition position) {
    return MathUtil.isNear(position.value, getPosition(), ShoulderConfig.CLOSED_LOOP_TOLERANCE_WIDE);
  }

  public Command waitUntilAtSetpoint() {
    return Commands.waitUntil(() -> isAtSetPoint()).withTimeout(ShoulderConfig.POSITION_CLOSED_LOOP_TIME_OVERRIDE);
  }

  public Command waitUntilNearSetpoint() {
    return Commands.waitUntil(() -> isNearSetPoint()).withTimeout(ShoulderConfig.POSITION_CLOSED_LOOP_TIME_OVERRIDE);
  }

  public Command positionCycleTest() {
    return Commands.sequence(
        setMotionMagicDutyCycle(EShoulerPosition.SPEAKER),
        setMotionMagicDutyCycle(EShoulerPosition.SPEAKER_PODIUM),
        setMotionMagicDutyCycle(EShoulerPosition.HORIZONTAL),
        setMotionMagicDutyCycle(EShoulerPosition.AMP),
        setMotionMagicDutyCycle(EShoulerPosition.HOME))
        .withName("PositionCycleTest");
  }

  private double getPosition() {
    return positionSupplier.get();
  }

  private boolean isAtSetPoint() {
    return MathUtil.isNear(rightRotator.getClosedLoopReference().getValueAsDouble(), getPosition(),
        ShoulderConfig.CLOSED_LOOP_TOLERANCE);
  }

  private boolean isNearSetPoint() {
    return MathUtil.isNear(rightRotator.getClosedLoopReference().getValueAsDouble(), getPosition(),
        ShoulderConfig.CLOSED_LOOP_TOLERANCE_WIDE);
  }

  private boolean isForwardLimit() {
    return MathUtil.isNear(EShoulerPosition.AMP.value, getPosition(), ShoulderConfig.CLOSED_LOOP_TOLERANCE);
  }

  private boolean isReverseLimit() {
    return MathUtil.isNear(EShoulerPosition.HOME.value, getPosition(), ShoulderConfig.CLOSED_LOOP_TOLERANCE);
  }
}
