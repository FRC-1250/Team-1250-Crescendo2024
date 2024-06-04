package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;

public class CANCoderPerformanceMonitor {

  private final List<DoublePerformanceCounter> doubleCounters;
  private final DoublePerformanceCounter velocity;
  private final DoublePerformanceCounter supplyVoltage;
  private final DoublePerformanceCounter absolutePosition;
  private final DoublePerformanceCounter positionSinceBoot;
  private final DoublePerformanceCounter magnetHealth;

  private final List<BooleanPerformanceCounter> booleanCounters;
  private final BooleanPerformanceCounter badMagnet;
  private final BooleanPerformanceCounter bootDuringEnable;
  private final BooleanPerformanceCounter hardware;
  private final BooleanPerformanceCounter underVoltage;

  public CANCoderPerformanceMonitor(
      CANcoder cancoder,
      String subsystemName,
      String deviceName) {
    velocity = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "Velocity",
        cancoder.getVelocity().asSupplier());
    supplyVoltage = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "SupplyVoltage",
        cancoder.getSupplyVoltage().asSupplier());
    absolutePosition = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "AbsolutePosition",
        cancoder.getAbsolutePosition().asSupplier());
    positionSinceBoot = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "PositionSinceBoot",
        cancoder.getPositionSinceBoot().asSupplier());
    magnetHealth = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "MagnetHealth",
        () -> {
          var c = cancoder.getMagnetHealth();
          c.refresh();
          return c.getValueAsDouble();
        });

    doubleCounters = new ArrayList<>();
    doubleCounters.add(velocity);
    doubleCounters.add(supplyVoltage);
    doubleCounters.add(absolutePosition);
    doubleCounters.add(positionSinceBoot);
    doubleCounters.add(magnetHealth);

    badMagnet = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultBadMagnet",
        cancoder.getFault_BadMagnet().asSupplier());
    bootDuringEnable = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultBootDuringEnable",
        cancoder.getFault_BootDuringEnable().asSupplier());
    hardware = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultHardware",
        cancoder.getFault_Hardware().asSupplier());
    underVoltage = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultUnderVoltage",
        cancoder.getFault_Undervoltage().asSupplier());

    booleanCounters = new ArrayList<>();
    booleanCounters.add(badMagnet);
    booleanCounters.add(bootDuringEnable);
    booleanCounters.add(hardware);
    booleanCounters.add(underVoltage);
  }

  public void telemeterize() {
    for (DoublePerformanceCounter counter : doubleCounters) {
      counter.push();
    }

    for (BooleanPerformanceCounter counter : booleanCounters) {
      counter.push();
    }
  }
}