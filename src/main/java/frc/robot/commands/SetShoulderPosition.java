// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Shoulder.Position;
import frc.robot.util.RobotHelper;

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
    shoulder.setPosition(targetPosition);
  }

  @Override
  public boolean isFinished() {
    return RobotHelper.isWithinRangeOfTarget(shoulder.getPosition(), targetPosition, 0.025);
  }
}
