// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SystemLights extends SubsystemBase {
  /** Creates a new SystemLights. */
  private final CANdle candle = new CANdle(0); 
    private final int LedCount = 300;
  public SystemLights(CANdle candle) {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.disableWhenLOS = false;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
  }

  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  public void SetLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
