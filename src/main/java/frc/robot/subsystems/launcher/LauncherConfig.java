package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LauncherConfig {
  public static final String SUBSYTEM_NAME = "Launcher";
  public static final String CAN_BUS = "rio";
  public static final int LEFT_LAUNCHER_CAN_ID = 20;
  public static final String LEFT_LAUNCHER_STRING = "Left launcher";
  public static final int RIGHT_LAUNCHER_CAN_ID = 21;
  public static final String RIGHT_LAUNCHER_STRING = "Right launcher";
  public static final int INDEXER_CAN_ID = 22;
  public static final String INDEXER_STRING = "Indexer";
  public static final int VELOCITY_CLOSED_LOOP_TOLERNACE = 100;
  public static final double FIRE_TIME_OVERRIDE = 0.5;

  private final static CurrentLimitsConfigs LAUNCHER_CURRENT_LIMITS = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(50);

  private final static MotorOutputConfigs LAUNCHER_MOTOR_OUTPUT = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.Clockwise_Positive);

  private final static Slot0Configs LAUNCHER_PID = new Slot0Configs()
      .withKP(0.11)
      .withKI(0)
      .withKD(0.0001)
      .withKV(0.115);

  public final static TalonFXConfiguration LAUNCHER_TALON_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(LAUNCHER_CURRENT_LIMITS)
      .withMotorOutput(LAUNCHER_MOTOR_OUTPUT)
      .withSlot0(LAUNCHER_PID);

  private final static CurrentLimitsConfigs INDEXER_CURRENT_LIMITS = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(20);

  private final static MotorOutputConfigs INDEXER_MOTOR_OUTPUT = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake);

  public final static TalonFXConfiguration INDEXER_TALON_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(INDEXER_CURRENT_LIMITS)
      .withMotorOutput(INDEXER_MOTOR_OUTPUT);
}
