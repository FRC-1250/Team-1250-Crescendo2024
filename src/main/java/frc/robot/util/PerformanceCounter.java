package frc.robot.util;

public class PerformanceCounter {
  protected final String dashboardPath;

  public PerformanceCounter(
      String subsystemName,
      String deviceName,
      String counterName) {
    this.dashboardPath = String.format("%s/%s/%s", subsystemName, deviceName, counterName);
  }
}
