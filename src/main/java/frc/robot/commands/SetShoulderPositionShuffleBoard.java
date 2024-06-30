// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.Shoulder.Position;

public class SetShoulderPositionShuffleBoard extends InstantCommand {
  private final Shoulder shoulder;

  public SetShoulderPositionShuffleBoard(Shoulder shoulder) {
    this.shoulder = shoulder;
    addRequirements(shoulder);
  }

  @Override
  public void initialize() {
    var pos = SmartDashboard.getNumber("Launcher/tuning pos", Position.HOME.value);
    if (pos <= Position.HOME.value) {
      pos = Position.HOME.value;
    } else if (pos >= Position.AMP.value) {
      pos = Position.AMP.value;
    }
    shoulder.setPositionDutyCycle(pos);
  }
}
