package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;

public class TalonFXPerformanceMonitor {

  private final List<DoublePerformanceCounter> doubleCounters;
  private final DoublePerformanceCounter velocity;
  private final DoublePerformanceCounter supplyVoltage;
  private final DoublePerformanceCounter acceleration;
  private final DoublePerformanceCounter statorCurrent;
  private final DoublePerformanceCounter supplyCurrent;
  private final DoublePerformanceCounter torqueCurrent;
  private final DoublePerformanceCounter position;

  private final List<BooleanPerformanceCounter> booleanCounters;
  private final BooleanPerformanceCounter bootDuringEnabled;
  private final BooleanPerformanceCounter brownout;
  private final BooleanPerformanceCounter hardware;
  private final BooleanPerformanceCounter overSupplyVoltage;
  private final BooleanPerformanceCounter remoteSensorInvalidData;
  private final BooleanPerformanceCounter remoteSensorReset;
  private final BooleanPerformanceCounter underSupplyVoltage;
  private final BooleanPerformanceCounter unstableSupplyVoltage;

  public TalonFXPerformanceMonitor(
      TalonFX talonfx,
      String subsystemName,
      String deviceName) {
    velocity = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "Velocity",
        talonfx.getVelocity().asSupplier());
    supplyVoltage = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "SupplyVoltage",
        talonfx.getSupplyVoltage().asSupplier());
    acceleration = new DoublePerformanceCounter(
        subsystemName,
        deviceName, "Acceleration",
        talonfx.getAcceleration().asSupplier());
    statorCurrent = new DoublePerformanceCounter(
        subsystemName,
        deviceName, "StatorCurrent",
        talonfx.getStatorCurrent().asSupplier());
    supplyCurrent = new DoublePerformanceCounter(
        subsystemName,
        deviceName, "SupplyCurrent",
        talonfx.getSupplyCurrent().asSupplier());
    torqueCurrent = new DoublePerformanceCounter(
        subsystemName,
        deviceName, "TorqueCurrent",
        talonfx.getTorqueCurrent().asSupplier());
    position = new DoublePerformanceCounter(
        subsystemName,
        deviceName,
        "Position",
        talonfx.getPosition().asSupplier());

    doubleCounters = new ArrayList<>();
    doubleCounters.add(velocity);
    doubleCounters.add(supplyVoltage);
    doubleCounters.add(acceleration);
    doubleCounters.add(statorCurrent);
    doubleCounters.add(supplyCurrent);
    doubleCounters.add(torqueCurrent);
    doubleCounters.add(position);

    bootDuringEnabled = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultBootDuringEnable",
        talonfx.getFault_BootDuringEnable().asSupplier());
    brownout = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultBrownout",
        talonfx.getFault_BootDuringEnable().asSupplier());
    hardware = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultHardware",
        talonfx.getFault_Hardware().asSupplier());
    overSupplyVoltage = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultOverSupplyVoltage",
        talonfx.getFault_OverSupplyV().asSupplier());
    remoteSensorInvalidData = new BooleanPerformanceCounter(
        subsystemName,
        deviceName,
        "FaultInvalidRemoteSensorData",
        talonfx.getFault_RemoteSensorDataInvalid().asSupplier());
    remoteSensorReset = new BooleanPerformanceCounter(
        subsystemName,
        deviceName, "FaultRemoteSensorReset",
        talonfx.getFault_RemoteSensorReset().asSupplier());
    underSupplyVoltage = new BooleanPerformanceCounter(
        subsystemName,
        deviceName, "FaultUnderSupplyVoltage",
        talonfx.getFault_Undervoltage().asSupplier());
    unstableSupplyVoltage = new BooleanPerformanceCounter(
        subsystemName,
        deviceName, "FaultUnstableSupplyVoltage",
        talonfx.getFault_UnstableSupplyV().asSupplier());

    booleanCounters = new ArrayList<>();
    booleanCounters.add(bootDuringEnabled);
    booleanCounters.add(brownout);
    booleanCounters.add(hardware);
    booleanCounters.add(overSupplyVoltage);
    booleanCounters.add(remoteSensorInvalidData);
    booleanCounters.add(remoteSensorReset);
    booleanCounters.add(underSupplyVoltage);
    booleanCounters.add(unstableSupplyVoltage);
  }

  public void telemeterize() {
      try {
          for (DoublePerformanceCounter doublePerformanceCounter : doubleCounters) {
              doublePerformanceCounter.push();
          }

          for (BooleanPerformanceCounter booleanPerformanceCounter : booleanCounters) {
              booleanPerformanceCounter.push();
          }
      } catch (Exception e) {
          DataLogManager.log(String.format("GatorBot: Not able to process TALON telemetry: %s", e.getMessage()));
      }
  }
}