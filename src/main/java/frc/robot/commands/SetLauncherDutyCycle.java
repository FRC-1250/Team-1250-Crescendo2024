// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class SetLauncherDutyCycle extends Command {
  private final Launcher launcher;
  private double percentOut;

  public SetLauncherDutyCycle(Launcher launcher, double percentOut) {
    this.launcher = launcher;
    this.percentOut = percentOut;
    addRequirements(launcher);
  }

  @Override
  public void execute() {
    launcher.SetDutyOutlaunch(percentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      launcher.SetDutyOutlaunch(0);
    }
  }
}
