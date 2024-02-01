// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer;

public class SetDutyCycleIndex extends Command {
  /** Creates a new SetDutyCycleIndex. */
  private final indexer indexer;
  private double percentOut;

  public SetDutyCycleIndex(indexer indexer, double percentOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.percentOut = percentOut;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
indexer.setDutyoutIndex(percentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   indexer.setDutyoutIndex(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
