// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SystemLights;

public class LightShow extends Command {

  private final SystemLights systemLights;
  private final BooleanSupplier hasNoteSupplier;
  private final BooleanSupplier isShoulderHomeSupplier;
  private Timer timer;
  private double LED_CYCLE = 0.1; // 100 ms

  public LightShow(SystemLights systemLights, BooleanSupplier hasNoteSupplier,
      BooleanSupplier isShoulderHomeSupplier) {
    this.systemLights = systemLights;
    this.hasNoteSupplier = hasNoteSupplier;
    this.isShoulderHomeSupplier = isShoulderHomeSupplier;
    timer = new Timer();
    addRequirements(systemLights);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.advanceIfElapsed(LED_CYCLE)) {
      if (hasNoteSupplier.getAsBoolean()) {
        systemLights.setLEDs(0, 0, 0);
      } else if (isShoulderHomeSupplier.getAsBoolean())
        timer.reset();
    }
  }
}
