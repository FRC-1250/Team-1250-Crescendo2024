package frc.robot.subsystems.shoulder;

public enum EShoulerPosition {
  AMP(.358f),
  HORIZONTAL(.25f),
  SPEAKER_PODIUM(.158f),
  SPEAKER(.105f),
  HOME(.055f),
  PID(.194f);

  public final float value; // Default value is rotations

  EShoulerPosition(float value) {
    this.value = value;
  }
}