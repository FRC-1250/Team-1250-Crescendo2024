package frc.robot.subsystems.leds;

public enum EColorProfile {
  RED(255, 0, 0),
  GREEN(0, 255, 0),
  BLUE(0, 0, 255);

  public final int r;
  public final int g;
  public final int b;

  EColorProfile(int r, int g, int b) {
    this.r = r;
    this.g = g;
    this.b = b;
  }
}
