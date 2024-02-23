// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shoulder.Position;

public class SystemLights extends SubsystemBase {
  /** Creates a new SystemLights. */
  private final indexer indexer;
  private final Shoulder shoulder;
  private final CANdle candle = new CANdle(0); 
  private final int LedCount = 300;


  public double getVbat() { return candle.getBusVoltage(); }
  public double get5V() { return candle.get5VRailVoltage(); }
  public double getCurrent() { return candle.getCurrent(); }
  public double getTemperature() { return candle.getTemperature(); }
  public void configBrightness(double percent) { candle.configBrightnessScalar(percent, 0); }
  public void configLos(boolean disableWhenLos) { candle.configLOSBehavior(disableWhenLos, 0); }
  public void configLedType(LEDStripType type) { candle.configLEDType(type, 0); }
  public void configStatusLedBehavior(boolean offWhenActive) { candle.configStatusLedState(offWhenActive, 0); }


  public SystemLights(CANdle candle, indexer indexer, Shoulder shoulder) {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.disableWhenLOS = false;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
    this.indexer = indexer;
    this.shoulder = shoulder;

    
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
    if(indexer.iscentered() == true) {
      setLEDs(255, 234, 0);
    }
    if(MathUtil.isNear(Position.HOME.value, shoulder.getPosition(), 0.005, 0, 1)) {
      setLEDs(170, 255, 0);
    } else {
      setLEDs(160, 34, 240);
    }
  }
}
