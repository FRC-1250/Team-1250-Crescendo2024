// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoulder.Shoulder;

public class SetShoulderDutyCycle extends Command {
  /** Creates a new SetShoulderDutyCycle. */
  private final Shoulder shoulder;
  private double percentOut;
  public SetShoulderDutyCycle(Shoulder shoulder, double percentOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoulder = shoulder;
    this.percentOut = percentOut;
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setDutyCycle(percentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulder.setDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
