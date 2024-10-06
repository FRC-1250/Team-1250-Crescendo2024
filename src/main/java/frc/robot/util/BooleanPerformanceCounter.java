package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BooleanPerformanceCounter extends PerformanceCounter {
  private final Supplier<Boolean> supplier;

  public BooleanPerformanceCounter(
      String subsystemName,
      String deviceName,
      String counterName,
      Supplier<Boolean> supplier) {
    super(subsystemName, deviceName, counterName);
    this.supplier = supplier;
  }

  public void push() {
    SmartDashboard.putBoolean(dashboardPath, supplier.get());
  }
}
