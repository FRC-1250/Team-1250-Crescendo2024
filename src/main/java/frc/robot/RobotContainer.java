// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FieldCentricAutoAim;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveConfig;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConfig;
import frc.robot.subsystems.launcher.ELauncherSpeed;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherConfig;
import frc.robot.subsystems.leds.SystemLights;
import frc.robot.subsystems.shoulder.EShoulerPosition;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderConfig;

public class RobotContainer {
  private final Intake intake = new Intake();
  private final Shoulder shoulder = new Shoulder();
  private final Launcher launcher = new Launcher();
  private final Indexer indexer = new Indexer();
  private final SystemLights systemLights = new SystemLights();
  private final CommandSwerveDrivetrain drivetrain = SwerveConfig.DriveTrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final CommandFactory cmdFactory = new CommandFactory(intake, shoulder, launcher,
      indexer, systemLights, drivetrain);

  // Field centric driving in closed loop with 10% deadband
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConfig.MaxSpeed * 0.1)
      .withRotationalDeadband(SwerveConfig.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(SwerveConfig.MaxSpeed * 0.1)
      .withRotationalDeadband(SwerveConfig.MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final FieldCentricAutoAim fieldCentricAutoAim = new FieldCentricAutoAim()
      .withDeadband(SwerveConfig.MaxSpeed * 0.1)
      .withRotationalDeadband(SwerveConfig.MaxAngularRate * 0.025)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer() {
    configureNamedCommands();
    configureAutoCommands();
    configureBindings();
    addSubsystemsToNT();
    CommandScheduler.getInstance().schedule(cmdFactory.visionOdometryUpdate());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
            .withVelocityX(
                -xboxController.getLeftY() * SwerveConfig.MaxSpeed * driveInvert * SwerveConfig.ThrottleValue)
            .withVelocityY(
                -xboxController.getLeftX() * SwerveConfig.MaxSpeed * driveInvert * SwerveConfig.ThrottleValue)
            .withRotationalRate(-xboxController.getRightX() * SwerveConfig.MaxAngularRate),
        "Default drive"));

    xboxController.x().whileTrue(drivetrain.applyRequestWithName(
        () -> fieldCentricAutoAim
            .withVelocityX(-xboxController.getLeftY() * SwerveConfig.MaxSpeed * driveInvert)
            .withVelocityY(-xboxController.getLeftX() * SwerveConfig.MaxSpeed * driveInvert),
        "Target lock"));

    xboxController.rightStick().whileTrue(drivetrain.applyRequestWithName(
        () -> robotCentricDrive
            .withVelocityX(-xboxController.getLeftY() * SwerveConfig.MaxSpeed * 0.5)
            .withVelocityY(-xboxController.getLeftX() * SwerveConfig.MaxSpeed * 0.5)
            .withRotationalRate(-xboxController.getRightX() * SwerveConfig.MaxAngularRate * 0.75),
        "Robot centric drive"));
  }

  private void addSubsystemsToNT() {
    SmartDashboard.putData(IntakeConfig.SUBSYTEM_NAME + "/Command", intake);
    SmartDashboard.putData(ShoulderConfig.SUBSYSTEM_NAME + "/Command", shoulder);
    SmartDashboard.putData(LauncherConfig.SUBSYTEM_NAME + "/Command", launcher);
    SmartDashboard.putData(IndexerConfig.SUBSYTEM_NAME + "/Command", indexer);
  }

  private Command getPathAuto(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  private void configureBindings() {
    systemLights.setDefaultCommand(cmdFactory.lightSignals());
    xboxController.leftBumper().onTrue(cmdFactory.prepFireNote(ELauncherSpeed.AMP, EShoulerPosition.AMP));
    xboxController.leftTrigger().onTrue(cmdFactory.prepFireNote(ELauncherSpeed.SPEAKER, EShoulerPosition.SPEAKER));
    xboxController.rightBumper().onTrue(cmdFactory.intakeNote());
    xboxController.rightTrigger().whileTrue(cmdFactory.fireNote()).onFalse(cmdFactory.startingConfiguration());
    xboxController.a().whileTrue(cmdFactory.unjam()).onFalse(cmdFactory.afterJam());
    xboxController.b().onTrue(cmdFactory.prepFireNote(ELauncherSpeed.PASS, EShoulerPosition.PASS));
    xboxController.pov(0).whileTrue(shoulder.setDutyCycle(0.75)).onFalse(shoulder.setDutyCycle(0));
    xboxController.pov(180).whileTrue(shoulder.setDutyCycle(-0.75)).onFalse(shoulder.setDutyCycle(0));
    xboxController.y().onTrue(cmdFactory.startingConfiguration());
    xboxController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
     SmartDashboard.putData("Intake Test", cmdFactory.intakeNote());
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
    autoChooser.addOption("FireNoteOnly",
        cmdFactory.autoFiringSequence(ELauncherSpeed.SPEAKER, EShoulerPosition.SPEAKER));
    addPathAuto("SpeakerCenter", "Center");
    addPathAuto("TestDrive", "TestDrive");
    addPathAuto("SpeakerCenterWithPodiumNote", "CenterWithPodiumNote");
    addPathAuto("SpeakerCenterWithAmpNote", "CenterWithAmpNote");
    addPathAuto("SpeakerALLNOTE", "CenterAllNote");
    addPathAuto("SpeakerAmpSide", "AmpSide");
    addPathAuto("SpeakerPodiumSide", "PodiumSide");
    addPathAuto("SourceDisrupt", "SourceSideDisrupt");
    addPathAuto("AmpESCAPE", "AmpESCAPE");
    addPathAuto("PodiumESCAPE", "PodiumESCAPE");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("intakeCenterNote", cmdFactory.intakeNote());
    NamedCommands.registerCommand("speakerFireNote",
        cmdFactory.autoFiringSequence(ELauncherSpeed.SPEAKER, EShoulerPosition.SPEAKER));
  }
}