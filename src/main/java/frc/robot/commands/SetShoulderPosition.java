// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shoulder.Position;

public class SetShoulderPosition extends Command {
  private final Shoulder shoulder;
  private final float position;

  public SetShoulderPosition(Shoulder shoulder, float position) {
    addRequirements(shoulder);
    this.shoulder = shoulder;
    this.position = position;
  }

  public SetShoulderPosition(Shoulder shoulder, Position position) {
    this(shoulder, position.value);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shoulder.setPosition(position);
  }

  @Override
  public boolean isFinished() {
    return shoulder.isAtSetPoint(position);
  }
}
