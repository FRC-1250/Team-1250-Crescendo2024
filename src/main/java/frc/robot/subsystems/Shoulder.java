// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCenterNote;
import frc.robot.util.CANCoderPerformanceMonitor;
import frc.robot.util.TalonFXPerformanceMonitor;

public class Shoulder extends SubsystemBase {

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
  private final double ENCODER_OFFSET = 0.0233; // Offset value to normalize the encoder position to 0 when at home
  private final NeutralOut BRAKE = new NeutralOut();
  private final TalonFX leftRotator;
  private final TalonFX rightRotator;
  private final CANcoder cancoder;
  private final DutyCycleOut dutyCycleOut;
  private final PositionDutyCycle positionDutyCycle;
  private final Supplier<Double> positionSupplier;
  private final CANCoderPerformanceMonitor canCoderPerformanceMonitor;
  private final TalonFXPerformanceMonitor rightRotatorMonitor;
  private final TalonFXPerformanceMonitor leftRotatorMonitor;

  public Shoulder() {
    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    configuration.MagnetSensor.MagnetOffset = ENCODER_OFFSET;
    cancoder = new CANcoder(CANCODER_CAN_ID, "rio");
    cancoder.getConfigurator().apply(configuration);
    positionSupplier = cancoder.getPosition().asSupplier();
    canCoderPerformanceMonitor = new CANCoderPerformanceMonitor(cancoder, getSubsystem(), "CANCoder");
    BaseStatusSignal.setUpdateFrequencyForAll(200, cancoder.getPosition(), cancoder.getVelocity());

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    // Configure the CANCoder as our feedback device
    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    talonFXConfiguration.Feedback.SensorToMechanismRatio = 1;

    // PID for the Shoulder
    talonFXConfiguration.Slot0.kP = 25;
    talonFXConfiguration.Slot0.kI = 0;
    talonFXConfiguration.Slot0.kD = 0.01;

    // Maxmimum amps supplied to the motors
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // The mode for which the motor is set to when not recieving a command
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.03;

    rightRotator = new TalonFX(RIGHT_ROTATOR_CAN_ID, "rio");
    rightRotator.getConfigurator().apply(talonFXConfiguration);
    rightRotatorMonitor = new TalonFXPerformanceMonitor(rightRotator, getSubsystem(), "RightRotator");

    leftRotator = new TalonFX(LEFT_ROTATOR_CAN_ID, "rio");
    leftRotator.getConfigurator().apply(talonFXConfiguration);
    leftRotator.setControl(new Follower(rightRotator.getDeviceID(), true));
    leftRotatorMonitor = new TalonFXPerformanceMonitor(leftRotator, getSubsystem(), "LeftRotator");

    dutyCycleOut = new DutyCycleOut(0);
    positionDutyCycle = new PositionDutyCycle(0);
  }

  public boolean isForwardLimit() {
    return MathUtil.isNear(Position.AMP.value, getPosition(), CLOSED_LOOP_TOLERANCE)
        || (getPosition() >= Position.AMP.value);
  }

  public boolean isReverseLimit() {
    return MathUtil.isNear(Position.HOME.value, getPosition(), CLOSED_LOOP_TOLERANCE)
        || (getPosition() <= Position.HOME.value);
  }

  public void setPosition(double targetPosition) {
    rightRotator.setControl(positionDutyCycle.withPosition(targetPosition)
        .withFeedForward(0.07)
        .withSlot(0)
        .withLimitForwardMotion(isForwardLimit())
        .withLimitReverseMotion(isReverseLimit()));
  }

  public void setDutyCycle(double percentOut) {
    rightRotator.setControl(dutyCycleOut.withOutput(percentOut)
        .withEnableFOC(true)
        .withLimitForwardMotion(isForwardLimit())
        .withLimitReverseMotion(isReverseLimit()));
  }

  public void stop() {
    rightRotator.setControl(BRAKE);
  }

  public double getPosition() {
    return positionSupplier.get();
  }

  public boolean isAtSetPoint(double targetPosition) {
    return MathUtil.isNear(targetPosition, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  public boolean isNearSetPoint(double targetPosition) {
    return MathUtil.isNear(targetPosition, getPosition(), CLOSED_LOOP_TOLERANCE * 7.5);
    // if we modify the scale value, we need to double check shooter positions to
    // check for overlap till the bounce is fixed
  }

  public boolean isAtHome() {
    return MathUtil.isNear(Position.HOME.value, getPosition(), CLOSED_LOOP_TOLERANCE);
  }

  public Command AmpCycleTest() {
    return Commands.sequence(
      //This is gonna go to the amp position for Prove OUT 
      setPositionAndWait(Position.AMP, 1)
    ).withName("Amp Cylce Test");
  }

  public Command SpeakerCycleTest() {
    return Commands.sequence(
      setPositionAndWait(Position.SPEAKER, 1)
    );
  }

  public Command PodiumCycleTest() {
    return Commands.sequence(
      setPositionAndWait(Position.SPEAKER_PODIUM, 1)
    );
  }

  public Command setPositionAndWait(Position position, double waitTimeSeconds) {
    return Commands.sequence(
          Commands.run(() -> setPosition(position.value), this).until(() -> isAtSetPoint(position.value)),
          Commands.waitSeconds(waitTimeSeconds));
  }

  public Command positionStop() {
    return Commands.sequence(
      Commands.runOnce(() -> stop(), this)
    ).withName("Stop");
  }

  @Override
  public void periodic() {
    rightRotatorMonitor.telemeterize();
    leftRotatorMonitor.telemeterize();
    canCoderPerformanceMonitor.telemeterize();
  }
}
