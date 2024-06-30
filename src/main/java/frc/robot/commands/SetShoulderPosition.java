// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.Shoulder.Position;

public class SetShoulderPosition extends Command {
  private final Shoulder shoulder;
  private final float targetPosition;

  public SetShoulderPosition(Shoulder shoulder, float targerPosition) {
    addRequirements(shoulder);
    this.shoulder = shoulder;
    this.targetPosition = targerPosition;
  }

  public SetShoulderPosition(Shoulder shoulder, Position targetPosition) {
    this(shoulder, targetPosition.value);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shoulder.setPositionDutyCycle(targetPosition);
  }

  public void end(boolean interrupted) {
    shoulder.setDutyCycle(0);
  }

  @Override
  public boolean isFinished() {
    return shoulder.isAtSetPoint(targetPosition);
  }
}
