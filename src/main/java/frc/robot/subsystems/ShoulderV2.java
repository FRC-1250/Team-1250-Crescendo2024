// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderV2 extends SubsystemBase {

  public enum Position {
    AMP(.358f),
    HORIZONTAL(.25f),
    SPEAKER_PODIUM(.158f),
    SPEAKER(.105f),
    HOME(.055f),
    PID(.194f);

    public final float value; // Default value is rotations

    Position(float value) {
      this.value = value;
    }
  }

  private final int LEFT_ROTATOR_CAN_ID = 30;
  private final int RIGHT_ROTATOR_CAN_ID = 31;
  private final int CANCODER_CAN_ID = 32;
  private final double CLOSED_LOOP_TOLERANCE = 0.003; // Closed loop tolerance in degrees
  private final double ENCODER_OFFSET = 0.134; // Offset value to normalize the encoder position to 0 when at home
  private final TalonFX leftRotator;
  private final TalonFX rightRotator;
  private final CANcoder cancoder;

  public ShoulderV2() {
    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    configuration.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
    cancoder = new CANcoder(CANCODER_CAN_ID, "rio");
    cancoder.getConfigurator().apply(configuration);
    cancoder.getPosition().setUpdateFrequency(100);

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;

    rightRotator = new TalonFX(RIGHT_ROTATOR_CAN_ID, "rio");
    rightRotator.getConfigurator().apply(talonFXConfiguration);

    leftRotator = new TalonFX(LEFT_ROTATOR_CAN_ID, "rio");
    leftRotator.getConfigurator().apply(new TalonFXConfiguration());
    leftRotator.setControl(new Follower(rightRotator.getDeviceID(), true));
  }

  public void setPosition(double targetPosition) {
  }

  public void setDutyCycle(double percentOut) {
    rightRotator.set(percentOut);
  }

  public void stop() {
    rightRotator.set(0);
  }

  public double getPosition() {
    return 0.0;
  }

  public boolean isAtSetPoint(double targetPosition) {
    return MathUtil.isNear(targetPosition, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  public boolean isAtHome() {
    return MathUtil.isNear(Position.HOME.value, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder/Absoulte position", cancoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shoulder/Degress position", cancoder.getPosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Shoulder/Right duty cycle", rightRotator.get());
    SmartDashboard.putNumber("Shoulder/Left duty cycle", leftRotator.get());
    SmartDashboard.putNumber("Shoulder/Right stator current", rightRotator.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shoulder/Left stator current", leftRotator.getStatorCurrent().getValueAsDouble());
  }
}
