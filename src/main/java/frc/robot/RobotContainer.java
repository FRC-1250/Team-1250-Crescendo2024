// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetShoulderDutyCycle;
import frc.robot.commands.SetShoulderPosition;
import frc.robot.commands.TargetLock;
import frc.robot.commands.FireNote;
import frc.robot.commands.IntakeCenterNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.launcher;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final Intake intake = new Intake();
  private final Shoulder shoulder = new Shoulder();
  private final launcher launcher = new launcher();
  private final indexer indexer = new indexer();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Limelight limelight = new Limelight();
   
  // Field centric driving in closed loop with 10% deadband
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic);

  // Field centric driving in closed loop with target locking and 10% deadband
  private final TargetLock targetLock = new TargetLock(limelight)
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagic);
  private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

  private final CommandXboxController drivXboxController = new CommandXboxController(0);
  // final CommandPS4Controller commandPS4Controller = new CommandPS4Controller(1);

  public RobotContainer() {
    configureAutoCommands();
    configureBindings();
  }

  private void configureBindings() {
    // Drive forward with -y, left with -x, rotate counter clockwise with -x
    drivetrain.setDefaultCommand(drivetrain.applyRequest(
        () -> drive
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed)
            .withRotationalRate(-drivXboxController.getRightX() * TunerConstants.MaxAngularRate)));

    drivXboxController.x().whileTrue(drivetrain.applyRequest(
        () -> targetLock
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed)));

    // reset the field-centric heading on left bumper press
    drivXboxController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivXboxController.rightBumper().onTrue(new IntakeCenterNote(intake, shoulder, indexer, -.5));
    drivXboxController.rightTrigger().whileTrue(new FireNote(indexer, launcher));
    drivXboxController.a().onTrue(new SetShoulderPosition(shoulder, 0.01f));
    drivXboxController.leftBumper().onTrue(new SetShoulderPosition(shoulder, 0.082f));
    drivXboxController.leftTrigger().onTrue(new SetShoulderPosition(shoulder, 0.2497f));
    drivXboxController.pov(0).whileTrue(new SetShoulderDutyCycle(shoulder, 0.5));
    drivXboxController.pov(180).whileTrue(new SetShoulderDutyCycle(shoulder, -0.5));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  private void configureAutoCommands() {
    /*
     * Do nothing as default is a human safety condition, this should always be the
     * default
     */
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
    try {
      autoChooser.addOption("SpeakerMiddle", getAutoPath("SpeakerMiddle"));
    } catch (Exception e) {
      System.out.println(String.format("%s", e.getCause()));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
