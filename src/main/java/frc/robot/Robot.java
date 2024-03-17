// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Alliance alliance;

  void checkDSUpdate() {
    Optional<Alliance> currentAllianceOpt = DriverStation.getAlliance();
    Alliance currentAlliance;

    if (currentAllianceOpt.isPresent()) {
      currentAlliance = currentAllianceOpt.get();
      if (DriverStation.isDSAttached() && currentAlliance != alliance) {
        alliance = currentAlliance;
        m_robotContainer.configureDrive(alliance);
      }
    }
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    checkDSUpdate();
    DataLogManager.start();

    CommandScheduler.getInstance().onCommandInitialize(
        command -> DataLogManager.log(
            String.format("Command init: %s, with requirements: %s", command.getName(), command.getRequirements())));

    CommandScheduler.getInstance().onCommandFinish(
        command -> DataLogManager.log(String.format("Command finished: %s", command.getName())));

    CommandScheduler.getInstance().onCommandInterrupt(
        command -> DataLogManager.log(String.format("Command interrupted: %s", command.getName())));

    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    checkDSUpdate();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    checkDSUpdate();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    checkDSUpdate();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    checkDSUpdate();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
