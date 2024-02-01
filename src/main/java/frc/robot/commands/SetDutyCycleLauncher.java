// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher;

public class SetDutyCycleLauncher extends Command {
  /** Creates a new SetDutyCycleLauncher. */
  private final launcher launcher;
  private double percentOut;
  
  public SetDutyCycleLauncher(launcher launcher, double percentOut) {
    this.launcher = launcher;
    this.percentOut = percentOut;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("initialize\n");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.SetDutyOutlaunch(percentOut);     
    System.out.print("execute " + percentOut + "\n");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.SetDutyOutlaunch(0);
    System.out.print("end\n");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.print("isFinished\n");
    return false;
  }
}
