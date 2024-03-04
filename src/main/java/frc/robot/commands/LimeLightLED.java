// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SystemLights;

public class LimeLightLED extends Command {

  private final BooleanSupplier hasNoteSupplier;
  private final BooleanSupplier isShoulderHomeSupplier;
  private final Limelight limelight;

  public LimeLightLED(Limelight limelight, BooleanSupplier hasNoteSupplier,
      BooleanSupplier isShoulderHomeSupplier) {
    this.limelight = limelight;
    this.hasNoteSupplier = hasNoteSupplier;
    this.isShoulderHomeSupplier = isShoulderHomeSupplier;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      if (hasNoteSupplier.getAsBoolean()) {
        limelight.setLEDMode(2);
      } else if (isShoulderHomeSupplier.getAsBoolean()) {
        limelight.setLEDMode(3);
      } else {
        limelight.setLEDMode(1);
      }
    
  }
}
