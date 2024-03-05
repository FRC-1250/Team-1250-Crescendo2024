// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
    // Drive forward with -y, left with -x, rotate counter clockwise with -
    //systemLights.setDefaultCommand(new LightShow(systemLights, indexer::iscentered, shoulder::isAtHome)); 
    limelight.setDefaultCommand(new LimeLightLED(limelight, indexer::iscentered, shoulder::isAtHome));
    drivetrain.setDefaultCommand(drivetrain.applyRequestWithName(
        () -> drive
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed)
            .withRotationalRate(-drivXboxController.getRightX() * TunerConstants.MaxAngularRate),
        "Default drive"));

    drivXboxController.x().whileTrue(drivetrain.applyRequestWithName(
        () -> fieldCentricAutoAim
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed),
        "Target lock"));
    
    drivXboxController.rightStick().whileTrue(drivetrain.applyRequestWithName(
            () -> robotCentricDrive
            .withVelocityX(-drivXboxController.getLeftY() * TunerConstants.MaxSpeed * 0.5)
            .withVelocityY(-drivXboxController.getLeftX() * TunerConstants.MaxSpeed * 0.5)
            .withRotationalRate(-drivXboxController.getRightX() * TunerConstants.MaxAngularRate * 0.75),
            "Robot centric drive"));


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

  private void configureAutoCommands() {
    /*
     * Do nothing as default is a human safety condition, this should always be the
     * default
     */
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
    try {
      autoChooser.addOption("FireNoteOnly", singleSpeakerShot());
      autoChooser.addOption("BlueSpeakerCenterShot", doubleSpeakerShot(HolonomicPaths.speakerCenter(Alliance.Blue)));
      autoChooser.addOption("BlueSpeakerAmpSideShot", doubleSpeakerShot(HolonomicPaths.speakerAmpSide(Alliance.Blue)));
      autoChooser.addOption("BlueSpeakerSourceSide", doubleSpeakerShot(HolonomicPaths.speakerSourceSide(Alliance.Blue)));
      autoChooser.addOption("RedSpeakerCenterShot", doubleSpeakerShot(HolonomicPaths.speakerCenter(Alliance.Red)));
      autoChooser.addOption("RedSpeakerAmpSideShot", doubleSpeakerShot(HolonomicPaths.speakerAmpSide(Alliance.Red)));
      autoChooser.addOption("RedSpeakerSourceSide", doubleSpeakerShot(HolonomicPaths.speakerSourceSide(Alliance.Red)));
         
    } catch (Exception e) {
      DataLogManager.log(String.format("GatorBot: Not able to build auto routines! %s", e.getMessage()));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Command singleSpeakerShot() {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
        fireNoteWithTimeout(),
        new SetShoulderPosition(shoulder, Position.HOME));
  }

  private Command doubleSpeakerShot(PathPlannerPath pathPlannerPath) {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value),
        fireNoteWithTimeout(),
        Commands.parallel(
            Commands.sequence(
                intakeCenterNoteWithFullSpeed(),
                new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER.value)),
            Commands.sequence(
                new WaitCommand(0.02),
                resetOdometryAndFollowPath(pathPlannerPath))),
        fireNoteWithTimeout(),
        new SetShoulderPosition(shoulder, Position.HOME));
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
    return new FireNote(indexer, launcher).withTimeout(0.5);
  }

  private Command intakeCenterNoteWithFullSpeed() {
    return new IntakeCenterNote(intake, shoulder, indexer, 1.0);
  }
}
