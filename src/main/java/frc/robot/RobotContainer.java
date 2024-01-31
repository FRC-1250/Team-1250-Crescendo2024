// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetDutyCycleIndex;
import frc.robot.commands.SetDutyCycleLauncher;
import frc.robot.commands.SetIntakeDutyCycle;
import frc.robot.commands.SetShoulderDutyCycle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.launcher;

public class RobotContainer {
  private final Intake intake = new Intake();
  private final Shoulder shoulder = new Shoulder();
  private final launcher launcher = new launcher();
  
  private final PS4Controller operatorPS4Controller = new PS4Controller(1);
  Trigger touchpad = new Trigger(operatorPS4Controller::getTouchpad);
  Trigger optionsButton = new Trigger(operatorPS4Controller::getOptionsButton);
  Trigger shareButton = new Trigger(operatorPS4Controller::getShareButton);
  Trigger crossButton = new Trigger(operatorPS4Controller::getCrossButton);
  Trigger triangleButton = new Trigger(operatorPS4Controller::getTriangleButton);
  Trigger squareButton = new Trigger(operatorPS4Controller::getSquareButton);
  Trigger circleButton = new Trigger(operatorPS4Controller::getCircleButton);
  Trigger psButton = new Trigger(operatorPS4Controller::getPSButton);
  Trigger l1Button = new Trigger(operatorPS4Controller::getL1Button);
  Trigger l2Button = new Trigger(operatorPS4Controller::getL2Button);
  Trigger l3Button = new Trigger(operatorPS4Controller::getL3Button);
  Trigger r1Button = new Trigger(operatorPS4Controller::getR1Button);
  Trigger r2Button = new Trigger(operatorPS4Controller::getR2Button);
  Trigger r3Button = new Trigger(operatorPS4Controller::getR3Button);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    triangleButton.onTrue(new SetIntakeDutyCycle(intake, -0.5));
    circleButton.onTrue(new SetIntakeDutyCycle(intake, 0));
    l1Button.whileTrue(new SetShoulderDutyCycle(shoulder, .2));
    l2Button.whileTrue(new SetShoulderDutyCycle(shoulder, -.2));
    r1Button.onTrue(new SetDutyCycleLauncher(launcher, .5));
    r2Button.onTrue(new SetDutyCycleLauncher(launcher, 0));
    crossButton.onTrue(new SetDutyCycleIndex(launcher, 0));
    squareButton.onTrue(new SetDutyCycleIndex(launcher, 0.5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
