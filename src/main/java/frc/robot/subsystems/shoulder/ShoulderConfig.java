package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ShoulderConfig {
  public static final String CAN_BUS = "rio";
  public static final String SUBSYSTEM_NAME = "Shoulder";
  public static final int LEFT_ROTATOR_CAN_ID = 30;
  public static final String LEFT_ROTATOR_STRING = "Left rotator";
  public static final int RIGHT_ROTATOR_CAN_ID = 31;
  public static final String RIGHT_ROTATOR_STRING = "Left rotator";
  public static final int CANCODER_CAN_ID = 32;
  public static final String CANCODER_STRING = "CANCoder";

  public static final double CLOSED_LOOP_TOLERANCE = 0.003; //1.08 degrees
  public static final double CLOSED_LOOP_TOLERANCE_WIDE = 0.0225; //8.1 degrees
  public static final double POSITION_CLOSED_LOOP_TIME_OVERRIDE = 1.2;

  private static final double ENCODER_OFFSET = 0.0233;
  private static final MagnetSensorConfigs MAGNET_SENSOR_CONFIGS = new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
      .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
      .withMagnetOffset(ENCODER_OFFSET);

  public static final CANcoderConfiguration CANCODER_CONFIGS = new CANcoderConfiguration()
      .withMagnetSensor(MAGNET_SENSOR_CONFIGS);

  private static final Slot0Configs PID = new Slot0Configs()
      .withKP(25)
      .withKI(0)
      .withKD(0.001);

  private static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(25);

  private static final MotorOutputConfigs MOTOR_OUTPUT = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withDutyCycleNeutralDeadband(0.03);

  private static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
      .withFeedbackRemoteSensorID(CANCODER_CAN_ID)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
      .withSensorToMechanismRatio(1);

  public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration()
      .withCurrentLimits(CURRENT_LIMITS)
      .withMotorOutput(MOTOR_OUTPUT)
      .withFeedback(FEEDBACK_CONFIGS)
      .withSlot0(PID);
}
