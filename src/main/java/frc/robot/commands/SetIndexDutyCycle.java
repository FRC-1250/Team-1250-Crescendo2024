// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class SetIndexDutyCycle extends Command {
  private final Indexer indexer;
  private double percentOut;

  public SetIndexDutyCycle(Indexer indexer, double percentOut) {
    this.indexer = indexer;
    this.percentOut = percentOut;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.setDutyoutIndex(percentOut);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setDutyoutIndex(0);
  }
}
