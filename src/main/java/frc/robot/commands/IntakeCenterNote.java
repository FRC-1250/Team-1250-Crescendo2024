// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.Shoulder.Position;
import frc.robot.util.RobotHelper;
import frc.robot.subsystems.Shoulder;

public class IntakeCenterNote extends Command {
  /** Creates a new CenterFireNote. */
  private final indexer indexer;
  private final Intake intake;
  private double percentOut;
  private final Shoulder shoulder;
  public IntakeCenterNote(Intake intake, Shoulder shoulder, indexer indexer, Double percentOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.indexer = indexer;
    this.percentOut = percentOut;
    this.shoulder = shoulder;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotHelper.isWithinRangeOfTarget(shoulder.getPosition(), Position.HOME.value, .025)) {
    intake.setDutyCycleUpperRoller(percentOut);
    intake.setDutyCycleLowerRoller(percentOut);
    indexer.centernote();
    } else {
      shoulder.setPosition(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  intake.setDutyCycleLowerRoller(0);
  intake.setDutyCycleUpperRoller(0);
  indexer.setDutyoutIndex(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return indexer.iscentered();
  }
}
