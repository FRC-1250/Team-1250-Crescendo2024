// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SetIntakeDutyCycle extends Command {
  private final Intake intake;
  private double percentOut;

  public SetIntakeDutyCycle(Intake intake, double percentOut) {
    this.intake = intake;
    this.percentOut = percentOut;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setDutyCycleRearRoller(percentOut);
    intake.setDutyCycleFrontRoller(percentOut);
  }
}
