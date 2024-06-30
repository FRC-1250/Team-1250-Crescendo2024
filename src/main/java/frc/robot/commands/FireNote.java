// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.ELauncherSpeed;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.Shoulder.Position;
import frc.robot.subsystems.superstructure.indexer;
import edu.wpi.first.wpilibj.Timer;

public class FireNote extends Command {
  private final Launcher launcher;
  private final indexer indexer;
  private final Shoulder shoulder;
  private final int CLOSED_LOOP_TOLERANCE = 100;
  private final double TRIGGER_TIME_OVERRIDE = 0.5;
  private final Timer timer = new Timer();

  /** Creates a new FireNote. */
  public FireNote(indexer indexer, Launcher launcher, Shoulder shoulder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcher = launcher;
    this.indexer = indexer; 
    this.shoulder = shoulder;
    addRequirements(launcher, indexer, shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetRPM = 0; 

    if (shoulder.isNearSetPoint(Position.SPEAKER.value)) {
      targetRPM = ELauncherSpeed.SPEAKER.value;
    } else if (shoulder.isNearSetPoint(Position.SPEAKER_PODIUM.value)) {
      targetRPM = ELauncherSpeed.PODIUM.value;
    } else {
      targetRPM = ELauncherSpeed.AMP.value;
    }

    launcher.setLauncherVelocity(targetRPM);

    if ((MathUtil.isNear(targetRPM, launcher.getLeftLauncherRPM(), CLOSED_LOOP_TOLERANCE)
        && MathUtil.isNear(targetRPM, launcher.getRightLauncherRPM(), CLOSED_LOOP_TOLERANCE))
        || timer.hasElapsed(TRIGGER_TIME_OVERRIDE)) {
      indexer.setDutyoutIndex(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setDutyoutIndex(0);
    launcher.setDutyCycleLauncher(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
