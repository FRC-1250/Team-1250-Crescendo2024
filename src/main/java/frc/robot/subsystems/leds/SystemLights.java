// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SystemLights extends SubsystemBase {
  /** Creates a new SystemLights. */
  private final CANdle candle;

  public SystemLights() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.disableWhenLOS = false;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    configAll.v5Enabled = false;
    candle = new CANdle(SystemLightsConfig.CANDLE_CAN_ID, SystemLightsConfig.CAN_BUS);
    candle.configAllSettings(configAll, 100);
  }

  public void setLeds(Animation animation) {
    candle.animate(animation);
  }

  public void setLeds(EColorProfile colorProfile) {
    candle.setLEDs(colorProfile.r, colorProfile.g, colorProfile.b);
  }
}
