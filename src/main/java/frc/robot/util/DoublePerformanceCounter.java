package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DoublePerformanceCounter {
  private final DoublePublisher publisher;
  private final Supplier<Double> supplier;
  private final String counterName;

  public DoublePerformanceCounter(
      String subsystemName,
      String deviceName,
      String counterName,
      Supplier<Double> supplier) {
    this.publisher = NetworkTableInstance.getDefault().getTable(subsystemName + "/" + deviceName)
        .getDoubleTopic(counterName).publish();
    this.supplier = supplier;
    this.counterName = counterName;
  }

  public void push() {
    publisher.set(supplier.get());
  }

  public String getCounterName() {
    return counterName;
  }
}
