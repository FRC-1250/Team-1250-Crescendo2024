// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.commands.FieldCentricAutoAim;
import frc.robot.commands.FireNote;
import frc.robot.commands.IntakeCenterNote;
import frc.robot.commands.LightShow;
import frc.robot.commands.LimeLightLED;
import frc.robot.commands.SetIndexDutyCycle;
import frc.robot.commands.SetIntakeDutyCycle;
import frc.robot.commands.SetLauncherDutyCycle;
import frc.robot.commands.SetPositionAndShooterSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.SystemLights;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.launcher;
import frc.robot.util.HolonomicPaths;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shoulder.Position;

public class RobotContainer {
  private final SystemLights systemLights = new SystemLights();
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
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final FieldCentricAutoAim fieldCentricAutoAim = new FieldCentricAutoAim(limelight)
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Field centric driving in closed loop with target locking and 10% deadband
  private final targetlock targetLock = new targetlock(limelight, drivetrain::getHeading)
      .withDeadband(TunerConstants.MaxSpeed * 0.1)
      .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.025)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandXboxController drivXboxController = new CommandXboxController(0);
  // final CommandPS4Controller commandPS4Controller = new CommandPS4Controller(1);

  public RobotContainer() {
    SmartDashboard.putData("Intake/Command", intake);
    SmartDashboard.putData("Shoulder/Command", shoulder);
    SmartDashboard.putData("Launcher/Command", launcher);
    SmartDashboard.putData("Indexer/Command", indexer);
    configureNamedCommands();
    configureAutoCommands();
    configureBindings();
  } 

  private void configureBindings() {
    // Drive forward with -y, left with -x, rotate counter clockwise with -
    //systemLights.setDefaultCommand(new LightShow(systemLights, indexer::iscentered, shoulder::isAtHome)); 
    limelight.setDefaultCommand(new LimeLightLED(limelight, indexer::iscentered, shoulder::isAtHome));
    
    drivXboxController.rightStick().whileTrue(drivetrain.applyRequestWithName(
            () -> robotCentricDrive
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed * 0.5)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed * 0.5)
            .withRotationalRate(-drivXboxController.getRightX() * TunerConstants.MaxAngularRate * 0.75),
            "Robot centric drive"));

    drivXboxController.rightBumper().onTrue(new IntakeCenterNote(intake, shoulder, indexer, 1.0));
    drivXboxController.rightTrigger().whileTrue(new FireNote(indexer, launcher, shoulder));
    drivXboxController.a().whileTrue(new SetIntakeDutyCycle(intake, -1));
    drivXboxController.a().whileTrue(new SetIndexDutyCycle(indexer, -.1));
    drivXboxController.b().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER_PODIUM));
    drivXboxController.leftTrigger().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER));
    drivXboxController.leftBumper().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.AMP));
    drivXboxController.pov(0).whileTrue(new SetShoulderDutyCycle(shoulder, 0.5));
    drivXboxController.pov(180).whileTrue(new SetShoulderDutyCycle(shoulder, -0.5));
    drivXboxController.y().onTrue(new SetIntakeDutyCycle(intake, 0));
    drivXboxController.y().onTrue(new SetLauncherDutyCycle(launcher, 0));
    drivXboxController.y().onTrue(new SetShoulderPosition(shoulder, Position.HOME.value));    
  }

  private void configureAutoCommands() {
    /*
     * Do nothing as default is a human safety condition, this should always be the
     * default
     */
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
    autoChooser.addOption("FireNoteOnly", singleSpeakerShot());
    try {
        autoChooser.addOption("SpeakerCenter", getPathAuto("Center"));
        autoChooser.addOption("SpeakerAmpSide", getPathAuto("AmpSide"));
        autoChooser.addOption("SpeakerPodiumSide", getPathAuto("PodiumSide"));
    } catch (Exception e) {
      DataLogManager.log(String.format("GatorBot: Not able to build auto routines! %s", e.getMessage()));
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getPathAuto(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("intakeCenterNote", new IntakeCenterNote(intake, shoulder, indexer, 1.0).withTimeout(1));
    NamedCommands.registerCommand("speakerFireNote", fireNoteWithTimeoutV2(Position.SPEAKER));
    NamedCommands.registerCommand("speakerShotPrep", new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER).withTimeout(1));
    NamedCommands.registerCommand("passingFireNote", fireNoteWithTimeoutV2(Position.SPEAKER_PODIUM));
    NamedCommands.registerCommand("passingShotPrep", new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER_PODIUM));
  }

  public void configureDrive(Alliance alliance) {
    double driveInvert;
    if (alliance == Alliance.Red) {
      driveInvert = -1;
    } else {
      driveInvert = 1;
    }
    drivetrain.setDefaultCommand(drivetrain.applyRequestWithName(
        () -> drive
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed * driveInvert)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed * driveInvert)
            .withRotationalRate(-drivXboxController.getRightX() * TunerConstants.MaxAngularRate),
        "Default drive"));

    drivXboxController.x().whileTrue(drivetrain.applyRequestWithName(
        () -> fieldCentricAutoAim
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed * driveInvert)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed * driveInvert),
        "Target lock"));

    if (alliance == Alliance.Red) {
      drivXboxController.start()
          .onTrue(drivetrain.runOnce(() -> drivetrain.setOdometry(Rotation2d.fromDegrees(180), new Pose2d())));
      drivetrain.setOdometry(Rotation2d.fromDegrees(180), new Pose2d());
    } else {
      drivXboxController.start()
          .onTrue(drivetrain.runOnce(() -> drivetrain.setOdometry(Rotation2d.fromDegrees(0), new Pose2d())));
      drivetrain.setOdometry(Rotation2d.fromDegrees(0), new Pose2d());
    }
  }

  private Command singleSpeakerShot() {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER),
        fireNoteWithTimeout(),
        new SetShoulderPosition(shoulder, Position.HOME));
  }

  private Command singleSpeakerShotWithPath(PathPlannerPath pathPlannerPath) {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER),
        fireNoteWithTimeout(),
        new SetShoulderPosition(shoulder, Position.HOME),
        resetOdometryAndFollowPath(pathPlannerPath));
  }

  private Command doubleSpeakerShot(PathPlannerPath pathPlannerPath) {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER),
        fireNoteWithTimeout(),
        Commands.parallel(
            Commands.sequence(
                intakeCenterNoteWithFullSpeed(),
                new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER)),
            Commands.sequence(
                new WaitCommand(0.02),
                resetOdometryAndFollowPath(pathPlannerPath))),
        fireNoteWithTimeout(),
        new SetShoulderPosition(shoulder, Position.HOME));
  }

  private Command doubleshotandrun(PathPlannerPath pathPlannerPath, PathPlannerPath pathPlannerPath2) {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER),
        fireNoteWithTimeout(),
        Commands.parallel(
            Commands.sequence(
                intakeCenterNoteWithFullSpeed(),
                new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER)),
            Commands.sequence(
                new WaitCommand(0.02),
                resetOdometryAndFollowPath(pathPlannerPath))),
        fireNoteWithTimeout(),
        new SetShoulderPosition(shoulder, Position.HOME),
        resetOdometryAndFollowPath(pathPlannerPath2));

  }

  public Command resetOdometryAndFollowPath(PathPlannerPath path) {
    return Commands.sequence(
        Commands.runOnce(
            () -> drivetrain.setOdometry(
                path.getPreviewStartingHolonomicPose().getRotation(),
                path.getPathPoses().get(0)),
            drivetrain),
        AutoBuilder.followPath(path));
  }

  private Command fireNoteWithTimeout() {
    return new FireNote(indexer, launcher, shoulder).withTimeout(1);
  }

  private Command intakeCenterNoteWithFullSpeed() {
    return new IntakeCenterNote(intake, shoulder, indexer, 1.0);
  }

  private Command doubleShotV2(PathPlannerPath path, Position position) {
    return Commands.sequence(
        fireNoteWithTimeoutV2(position),
        followPathWithResetAndIntake(path, position),
        fireNoteWithTimeoutV2(position));
  }

  private Command doubleShotAndRunV2(PathPlannerPath intakePath, PathPlannerPath run, Position position) {
    return Commands.sequence(
        fireNoteWithTimeoutV2(position),
        followPathWithResetAndIntake(intakePath, position),
        fireNoteWithTimeoutV2(position),
        followPath(run));
  }

  private Command tripleShot(PathPlannerPath intakePathOne, PathPlannerPath intakePathTwo, Position position) {
    return Commands.sequence(
        fireNoteWithTimeoutV2(position),
        followPathWithResetAndIntake(intakePathOne, position),
        fireNoteWithTimeoutV2(position),
        followPathAndIntake(intakePathTwo, position),
        fireNoteWithTimeoutV2(position));
  }

  private Command tripleShotAndRun(PathPlannerPath intakePathOne, PathPlannerPath intakePathTwo, PathPlannerPath run, Position position) {
    return Commands.sequence(
        fireNoteWithTimeoutV2(position),
        followPathWithResetAndIntake(intakePathOne, position),
        fireNoteWithTimeoutV2(position),
        followPathAndIntake(intakePathTwo, position),
        fireNoteWithTimeoutV2(position),
        followPath(run));
  }

  private Command fireNoteWithTimeoutV2(Position position) {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, position),
        new FireNote(indexer, launcher, shoulder).withTimeout(1));
  }

  private Command followPathWithResetAndIntake(PathPlannerPath path, Position position) {
    return Commands.parallel(
        Commands.sequence(
            new IntakeCenterNote(intake, shoulder, indexer, 1.0),
            new SetPositionAndShooterSpeed(shoulder, launcher, position)),
        followPathWithReset(path));
  }

  private Command followPathAndIntake(PathPlannerPath path, Position position) {
    return Commands.parallel(
        Commands.sequence(
            new IntakeCenterNote(intake, shoulder, indexer, 1.0),
            new SetPositionAndShooterSpeed(shoulder, launcher, position)),
        followPath(path));
  }

  private Command followPath(PathPlannerPath path) {
    return Commands.sequence(
        new WaitCommand(0.05),
        AutoBuilder.followPath(path));
  }

  private Command followPathWithReset(PathPlannerPath path) {
    return Commands.sequence(
        new WaitCommand(0.05),
        Commands.runOnce(
            () -> drivetrain.setOdometry(
                path.getPreviewStartingHolonomicPose().getRotation(),
                path.getPathPoses().get(0)),
            drivetrain),
        AutoBuilder.followPath(path));
  }
}
