package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoublePerformanceCounter extends PerformanceCounter {
  private final Supplier<Double> supplier;

  public DoublePerformanceCounter(
      String subsystemName,
      String deviceName,
      String counterName,
      Supplier<Double> supplier) {
    super(subsystemName, deviceName, counterName);
    this.supplier = supplier;
  }

  public void push() {
    SmartDashboard.putNumber(dashboardPath, supplier.get());
  }
}
