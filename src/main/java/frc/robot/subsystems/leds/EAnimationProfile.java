package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;

public enum EAnimationProfile {
  LARSON(new LarsonAnimation(EColorProfile.GREEN.r, EColorProfile.GREEN.g, EColorProfile.GREEN.b));

  public final Animation animate;

  EAnimationProfile(Animation animate) {
    this.animate = animate;
  }
}
