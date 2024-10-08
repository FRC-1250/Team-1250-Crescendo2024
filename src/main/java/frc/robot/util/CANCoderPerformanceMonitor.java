package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class CANCoderPerformanceMonitor {
  private final List<StatusSignal<?>> signals;
  private final BaseStatusSignal[] baseStatusSignals;
  private final String subsystemName;
  private final String deviceName;

  public CANCoderPerformanceMonitor(
      CANcoder cc,
      String subsystemName) {
    signals = new ArrayList<>();
    signals.add(cc.getAbsolutePosition());
    signals.add(cc.getVelocity());
    signals.add(cc.getMagnetHealth());
    signals.add(cc.getSupplyVoltage());
    signals.add(cc.getVelocity());
    signals.add(cc.getFault_BadMagnet());
    signals.add(cc.getFault_BootDuringEnable());
    signals.add(cc.getFault_Hardware());
    signals.add(cc.getFault_Undervoltage());

    this.subsystemName = subsystemName;
    this.deviceName = "CANcoder";
    this.baseStatusSignals = signals.toArray(BaseStatusSignal[]::new);
  }

  public void push() {
    if (signals.size() > 0) {
      StatusSignal.refreshAll(baseStatusSignals);
      for (StatusSignal<?> statusSignal : signals) {
        statusSignal.refresh(false);
        SmartDashboard.putNumber(String.format("%s/%s/%s", subsystemName, deviceName, statusSignal.getName()),
            statusSignal.getValueAsDouble());
      }
    }
  }
}
