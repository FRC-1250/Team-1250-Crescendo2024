// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetShoulderDutyCycle;
import frc.robot.commands.SetShoulderPosition;
import frc.robot.commands.targetlock;
import frc.robot.commands.FireNote;
import frc.robot.commands.IntakeCenterNote;
import frc.robot.commands.SetIntakeDutyCycle;
import frc.robot.commands.SetLauncherDutyCycle;
import frc.robot.commands.SetPositionAndShooterSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.launcher;
import frc.robot.util.HolonomicPathBuilder;
import frc.robot.util.HolonomicPaths;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shoulder.Position; 

public class RobotContainer {
  private final Intake intake = new Intake();
  private final Shoulder shoulder = new Shoulder();
  private final launcher launcher = new launcher();
  private final indexer indexer = new indexer();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Limelight limelight = new Limelight();
  private final HolonomicPaths hp = new HolonomicPaths();
  private final HolonomicPathBuilder pb = new HolonomicPathBuilder();
   
  // Field centric driving in closed loop with 10% deadband
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Field centric driving in closed loop with target locking and 10% deadband
  private final targetlock targetLock = new targetlock(limelight, drivetrain::getHeading)
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);

  private final CommandXboxController drivXboxController = new CommandXboxController(0);
  // final CommandPS4Controller commandPS4Controller = new CommandPS4Controller(1);

  public RobotContainer() {
    SmartDashboard.putData(intake);
    SmartDashboard.putData(shoulder);
    SmartDashboard.putData(launcher);
    SmartDashboard.putData(indexer);
    configureAutoCommands();
    configureBindings();
  }

  private void configureBindings() {
    // Drive forward with -y, left with -x, rotate counter clockwise with -x
    drivetrain.setDefaultCommand(drivetrain.applyRequestWithName(
        () -> drive
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed)
            .withRotationalRate(-drivXboxController.getRightX() * TunerConstants.MaxAngularRate), "Default drive"));

    drivXboxController.x().whileTrue(drivetrain.applyRequestWithName(
        () -> targetLock
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed), "Target lock"));

    // reset the field-centric heading on left bumper press
    drivXboxController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivXboxController.rightBumper().onTrue(new IntakeCenterNote(intake, shoulder, indexer, 1.0));
    drivXboxController.rightTrigger().whileTrue(new FireNote(indexer, launcher));
    drivXboxController.a().onTrue(new SetIntakeDutyCycle(intake, -1));
    drivXboxController.b().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER_PODIUM.value));
    drivXboxController.leftTrigger().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value));
    drivXboxController.leftBumper().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.AMP.value));
    drivXboxController.pov(0).whileTrue(new SetShoulderDutyCycle(shoulder, 0.5));
    drivXboxController.pov(180).whileTrue(new SetShoulderDutyCycle(shoulder, -0.5));
    drivXboxController.y().onTrue(new SetIntakeDutyCycle(intake, 0));
    drivXboxController.y().onTrue(new SetLauncherDutyCycle(launcher, 0));
    drivXboxController.y().onTrue(new SetShoulderPosition(shoulder, Position.HOME.value));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  public Command buildAutoCommand(PathPlannerPath path) {
    return Commands.sequence(
        Commands.runOnce(
            () -> drivetrain.setOdometry(
                path.getPreviewStartingHolonomicPose().getRotation(),
                path.getPathPoses().get(0)),
            drivetrain));
  }

  private void configureAutoCommands() {
    /*
     * Do nothing as default is a human safety condition, this should always be the
     * default
     */
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
    try {
      autoChooser.addOption("FireNoteOnly",
          Commands.sequence(
              new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME)));

      autoChooser.addOption("BlueSpeakerCenterShot",
          Commands.sequence(
              new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME),
              buildAutoCommand(pb.build(Alliance.Blue, hp.speakerCenterAndLeaveStartingZone))));

      autoChooser.addOption("BlueSpeakerAmpSideShot",
          Commands.sequence(
             new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME),
              buildAutoCommand(pb.build(Alliance.Blue, hp.speakerAmpSideAndLeaveStartingZone))));

      autoChooser.addOption("BlueSpeakerSourceSide",
          Commands.sequence(
              new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME),
              buildAutoCommand(pb.build(Alliance.Blue, hp.speakerSourceSideAndLeaveStartingZone))));

      autoChooser.addOption("RedSpeakerCenterShot",
          Commands.sequence(
              new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME),
              buildAutoCommand(pb.build(Alliance.Red, hp.speakerCenterAndLeaveStartingZone))));

      autoChooser.addOption("RedSpeakerAmpSideShot",
          Commands.sequence(
              new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME),
              buildAutoCommand(pb.build(Alliance.Red, hp.speakerAmpSideAndLeaveStartingZone))));

      autoChooser.addOption("RedSpeakerSourceSide",
          Commands.sequence(
              new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
              new FireNote(indexer, launcher).withTimeout(2),
              new SetShoulderPosition(shoulder, Position.HOME),
              buildAutoCommand(pb.build(Alliance.Red, hp.speakerSourceSideAndLeaveStartingZone))));
    } catch (Exception e) {
      System.out.println(String.format("GatorBot: Not able to build auto routines! %s", e.getCause()));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
