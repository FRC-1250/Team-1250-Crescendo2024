// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.launcher;
import frc.robot.subsystems.Shoulder.Position;
import edu.wpi.first.wpilibj.Timer;

public class FireNote extends Command {
  private final launcher launcher;
  private final indexer indexer;
  private final Shoulder shoulder;
  private final int CLOSED_LOOP_TOLERANCE = 100;
  Timer timer = new Timer();

  /** Creates a new FireNote. */
  public FireNote(indexer indexer, launcher launcher, Shoulder shoulder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.indexer = indexer; 
    this.shoulder = shoulder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var targetRPM = 0;
    var targetRPMleft = 0; 
    var targetRPMright = 0; 

    if (shoulder.isAtSetPoint(Position.SPEAKER.value) || shoulder.isAtSetPoint(Position.SPEAKER_PODIUM.value)) {
      targetRPMright = launcher.SPEAKER_TARGET_RPM_RIGHT;
      targetRPMleft = launcher.SPEAKER_TARGET_RPM_LEFT; 
    } else if (shoulder.isAtSetPoint(Position.SPEAKER_PODIUM.value)) {
      targetRPMright = launcher.PODIUM_TARGET_RPM_RIGHT;
      targetRPMleft = launcher.PODIUM_TARGET_RPM_LEFT;
    } else {
      targetRPMleft = launcher.AMP_TARGET_RPM_LEFT;
      targetRPMright = launcher.AMP_TARGET_RPM_RIGHT;
    }

    launcher.SetLauncherVelocity(targetRPMright, targetRPMleft);

    if (MathUtil.isNear(targetRPMleft,launcher.getLeftLauncherRPM(), CLOSED_LOOP_TOLERANCE) && MathUtil.isNear(targetRPMright, launcher.getRightLauncherRPM(), CLOSED_LOOP_TOLERANCE) || timer.hasElapsed(3)) {
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
