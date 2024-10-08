package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PigeonIMUPerformanceMonitor {
   private final List<StatusSignal<?>> signals;
  private final BaseStatusSignal[] baseStatusSignals;
  private final String subsystemName;
  private final String deviceName;

  public PigeonIMUPerformanceMonitor(
      Pigeon2 pigeon,
      String subsystemName) {
        signals = new ArrayList<>();
        signals.add(pigeon.getPitch());
        signals.add(pigeon.getRoll());
        signals.add(pigeon.getYaw());
        signals.add(pigeon.getSupplyVoltage());
        signals.add(pigeon.getStickyFault_BootDuringEnable());
        signals.add(pigeon.getStickyFault_Hardware());
        signals.add(pigeon.getStickyFault_Undervoltage());
    
        this.subsystemName = subsystemName;
        this.deviceName = "PigeonIMU";
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
