package frc.robot.temp;

import java.time.Instant;

public class DataPoint {
  private Instant timestamp;
  private Double value;

  public DataPoint(Instant timestamp, Double value) {
    this.timestamp = timestamp;
    this.value = value;
    value.doubleValue();
  }

  public Instant getTimestamp() {
    return timestamp;
  }

  public Double getValue() {
    return value;
  }

}
