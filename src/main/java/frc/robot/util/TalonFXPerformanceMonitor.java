package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFXPerformanceMonitor {
  private final List<StatusSignal<?>> signals;
  private final BaseStatusSignal[] baseStatusSignals;
  private final String subsystemName;
  private final String deviceName;

  public TalonFXPerformanceMonitor(
      TalonFX talonfx,
      String subsystemName,
      String deviceName) {
    signals = new ArrayList<>();
    signals.add(talonfx.getVelocity());
    signals.add(talonfx.getSupplyVoltage());
    signals.add(talonfx.getAcceleration());
    signals.add(talonfx.getStatorCurrent());
    signals.add(talonfx.getSupplyCurrent());
    signals.add(talonfx.getTorqueCurrent());
    signals.add(talonfx.getStickyFault_BootDuringEnable());
    signals.add(talonfx.getStickyFault_BridgeBrownout());
    signals.add(talonfx.getStickyFault_Hardware());
    signals.add(talonfx.getStickyFault_OverSupplyV());
    signals.add(talonfx.getStickyFault_RemoteSensorDataInvalid());
    signals.add(talonfx.getStickyFault_RemoteSensorReset());
    signals.add(talonfx.getStickyFault_Undervoltage());
    signals.add(talonfx.getStickyFault_UnstableSupplyV());

    this.subsystemName = subsystemName;
    this.deviceName = deviceName;
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