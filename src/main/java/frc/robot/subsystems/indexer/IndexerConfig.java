package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConfig {
    public static final String SUBSYTEM_NAME = "Indexer";
    public static final String CAN_BUS = "rio";
    public static final int CAN_ID = 22;
    public static final String INDEXER_STRING = "Indexer";

    private final static CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(20);

    private final static MotorOutputConfigs MOTOR_OUTPUT = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);

    private final static Slot0Configs VELOCITY_PID = new Slot0Configs()
            .withKP(0.11)
            .withKI(0)
            .withKD(0.0001)
            .withKV(0.115);

    public final static TalonFXConfiguration TALON_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(CURRENT_LIMITS)
            .withMotorOutput(MOTOR_OUTPUT)
            .withSlot0(VELOCITY_PID);

    public static final double STAGING_SPEED = 0.35;
    public static final double CENTERING_SPEED = 0.05;
}
