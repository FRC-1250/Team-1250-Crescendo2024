package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConfig {
  public final static String SUBSYTEM_NAME = "Intake";
  public final static int FRONT_ROLLER_CAN_ID = 40;
  public final static String FRONT_ROLLER_STRING = "Front roller";
  public final static int REAR_ROLLER_CAN_ID = 41;
  public final static String REAR_ROLLER_STRING = "Rear roller";

  private final static CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(25);

  private final static MotorOutputConfigs MOTOR_OUTPUT = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake);

  public final static TalonFXConfiguration TALON_CONFIG = new TalonFXConfiguration()
      .withCurrentLimits(CURRENT_LIMITS)
      .withMotorOutput(MOTOR_OUTPUT);

}
