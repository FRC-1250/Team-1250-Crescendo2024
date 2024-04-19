// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetShoulderDutyCycle;
import frc.robot.commands.SetShoulderPosition;
import frc.robot.commands.SetShoulderPositionShuffleBoard;
import frc.robot.commands.FieldCentricAutoAim;
import frc.robot.commands.FireNote;
import frc.robot.commands.IntakeCenterNote;
import frc.robot.commands.LimeLightLED;
import frc.robot.commands.SetIndexDutyCycle;
import frc.robot.commands.SetIntakeDutyCycle;
import frc.robot.commands.SetLauncherDutyCycle;
import frc.robot.commands.SetLauncherVelocityShuffleBoard;
import frc.robot.commands.SetPositionAndShooterSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.launcher;
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

  private final CommandXboxController drivXboxController = new CommandXboxController(0);

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
    SmartDashboard.putData(new SetShoulderPositionShuffleBoard(shoulder));
    SmartDashboard.putData(new SetLauncherVelocityShuffleBoard(launcher));
    SmartDashboard.putData("Start indexer", new SetIndexDutyCycle(indexer, 1));
    SmartDashboard.putData("Stop indexer", new SetIndexDutyCycle(indexer, 0));
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
    drivXboxController.a().whileTrue(new SetLauncherDutyCycle(launcher, -.5));
    drivXboxController.b().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER_PODIUM));
    drivXboxController.leftTrigger().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER));
    drivXboxController.leftBumper().onTrue(new SetPositionAndShooterSpeed(shoulder, launcher, Position.AMP));
    drivXboxController.pov(0).whileTrue(new SetShoulderDutyCycle(shoulder, 0.5));
    drivXboxController.pov(180).whileTrue(new SetShoulderDutyCycle(shoulder, -0.5));
    drivXboxController.y().onTrue(new SetIntakeDutyCycle(intake, 0));
    drivXboxController.y().onTrue(new SetLauncherDutyCycle(launcher, 0));
    drivXboxController.y().onTrue(new SetShoulderPosition(shoulder, Position.HOME.value));    
  }

  private void addPathAuto(String name, String pathName) {
    try {
      autoChooser.addOption(name, getPathAuto(pathName));
    } catch (Exception e) {
      DataLogManager.log(String.format("GatorBot: Not able to build auto routines! %s", e.getMessage()));
    }
  }

  private void configureAutoCommands() {
    /*
     * Do nothing as default is a human safety condition, this should always be the
     * default
     */
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
    autoChooser.addOption("FireNoteOnly", fireNoteWithTimeoutV2(Position.SPEAKER));
    addPathAuto("SpeakerCenter", "Center");
    addPathAuto("SpeakerCenterWithPodiumNote", "CenterWithPodiumNote");
    addPathAuto("SpeakerCenterWithAmpNote", "CenterWithAmpNote");
    addPathAuto("SpeakerAmpSide", "AmpSide");
    addPathAuto("SpeakerPodiumSide", "PodiumSide");
    addPathAuto("SpeakerAmpSideDisruptNotes", "AmpDisruptNotes");
    addPathAuto("SpeakerSourceDisrupt", "SourceSideDisrupt");
    addPathAuto("SpeakerALLNOTE", "CenterAllNote");
    addPathAuto("Poduim3Note", "Poduim3Note");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getPathAuto(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("intakeCenterNote", new IntakeCenterNote(intake, shoulder, indexer, 1.0));
    NamedCommands.registerCommand("speakerFireNote", fireNoteWithTimeoutV2(Position.SPEAKER));
    NamedCommands.registerCommand("speakerShotPrep", new SetPositionAndShooterSpeed(shoulder, launcher, Position.SPEAKER));
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

  private Command fireNoteWithTimeoutV2(Position position) {
    return Commands.sequence(
        new SetPositionAndShooterSpeed(shoulder, launcher, position),
        new FireNote(indexer, launcher, shoulder).withTimeout(0.7));
  }
}