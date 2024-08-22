// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.launcher;
import frc.robot.subsystems.Shoulder.Position;

public class SetPositionAndShooterSpeed extends Command {
  private final Shoulder shoulder;
  private final launcher launcher;
  private final Position targetPosition;
  /** Creates a new SetPositionAndShooterSpeed. */
  public SetPositionAndShooterSpeed(Shoulder shoulder, launcher launcher, Position targetPosition) {
    this.launcher = launcher;
    this.shoulder = shoulder;
    this.targetPosition = targetPosition;
    addRequirements(launcher, shoulder);




    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetPosition == Position.SPEAKER) {
      launcher.SetLauncherVelocity(launcher.SPEAKER_TARGET_RPM, launcher.SPEAKER_TARGET_RPM);
    } else if ( targetPosition == Position.SPEAKER_PODIUM) { 
      launcher.SetLauncherVelocity(launcher.PODIUM_TARGET_RPM, launcher.PODIUM_TARGET_RPM);
    }else if (targetPosition == Position.AMP) {
      launcher.SetLauncherVelocity(launcher.AMP_TARGET_RPM, launcher.AMP_TARGET_RPM);
    }
    shoulder.setPosition(targetPosition.value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shoulder.isAtSetPoint(targetPosition.value);
  }
}
