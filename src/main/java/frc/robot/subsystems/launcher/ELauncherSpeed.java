package frc.robot.subsystems.launcher;

public enum ELauncherSpeed {
  FALCON_500_MAX(6380),
  FALCON_500_MAX_FOC(6080),
  KRAKEN_MAX(6000),
  KRAKEN_MAX_FOC(5800),
  AMP(1500),
  PODIUM(5000),
  PASS(5000),
  SPEAKER(3500),
  IDLE(0);

  public final double value;

  ELauncherSpeed(final double value) {
    this.value = value;
  }
}
