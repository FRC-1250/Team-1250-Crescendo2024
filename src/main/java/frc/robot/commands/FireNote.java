// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.launcher;

public class FireNote extends Command {
  private final launcher launcher;
  private final indexer indexer;
  private final int TARGET_RPM = 5000;
  /** Creates a new FireNote. */
  public FireNote(indexer indexer, launcher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.indexer = indexer; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.SetLauncherVelocity(TARGET_RPM);
    if (MathUtil.isNear(TARGET_RPM, launcher.returnRPM(), 100)) {
      indexer.setDutyoutIndex(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setDutyoutIndex(0);
    launcher.SetDutyOutlaunch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
