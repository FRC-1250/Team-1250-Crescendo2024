// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.launcher;

public class SetLauncherVelocityShuffleBoard extends InstantCommand {
  private final launcher launcher;

  public SetLauncherVelocityShuffleBoard(launcher launcher) {
    this.launcher = launcher;
    addRequirements(launcher);
  }

  @Override
  public void initialize() {
    var rpm = SmartDashboard.getNumber("Launcher/tuning RPM", 0);
    launcher.SetLauncherVelocity(rpm, rpm);
  }
}
