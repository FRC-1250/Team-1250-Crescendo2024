package frc.robot.util;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.temp.DataPoint;

public class DoublePerformanceCounter {
  private final DoublePublisher publisher;
  private final Supplier<Double> supplier;
  private final String counterName;
  private final List<DataPoint> dataPoints;
  private boolean recordDataPoints;

  public DoublePerformanceCounter(
      String subsystemName,
      String deviceName,
      String counterName,
      Supplier<Double> supplier) {
    this.publisher = NetworkTableInstance.getDefault().getTable(subsystemName + "/" + deviceName)
        .getDoubleTopic(counterName).publish();
    this.supplier = supplier;
    this.counterName = counterName;
    this.dataPoints = new ArrayList<>();
    recordDataPoints = false;
  }

  public void push() {
    Double value = supplier.get();
    if (recordDataPoints) {
      dataPoints.add(new DataPoint(Instant.now(), value));
    }
    publisher.set(value);
  }

  public String getCounterName() {
    return counterName;
  }

  public void enableRecord() {
    dataPoints.clear();
    recordDataPoints = true;
  }

  public void disableRecord() {
    recordDataPoints = false;
  }
}
