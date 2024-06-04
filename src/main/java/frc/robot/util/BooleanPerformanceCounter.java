package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BooleanPerformanceCounter {
  private final BooleanPublisher publisher;
  private final Supplier<Boolean> supplier;
  private final String counterName;

  public BooleanPerformanceCounter(
      String subsystemName,
      String deviceName,
      String counterName,
      Supplier<Boolean> supplier) {
    this.publisher = NetworkTableInstance.getDefault().getTable(subsystemName + "/" + deviceName)
        .getBooleanTopic(counterName).publish();
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
