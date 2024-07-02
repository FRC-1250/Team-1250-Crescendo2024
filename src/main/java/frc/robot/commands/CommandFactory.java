package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.ELauncherSpeed;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.leds.EColorProfile;
import frc.robot.subsystems.leds.SystemLights;
import frc.robot.subsystems.shoulder.EShoulerPosition;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.vision.Limelight;

public class CommandFactory {

  private final Intake intake;
  private final Shoulder shoulder;
  private final Launcher launcher;
  private final Indexer indexer;
  private final SystemLights systemLights;
  private final Limelight limelight;
  private final CommandSwerveDrivetrain commandSwerveDrivetrain;

  public CommandFactory(Intake intake, Shoulder shoulder, Launcher launcher, Indexer indexer,
      SystemLights systemLights, Limelight limelight, CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.intake = intake;
    this.shoulder = shoulder;
    this.launcher = launcher;
    this.indexer = indexer;
    this.systemLights = systemLights;
    this.limelight = limelight;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
  }

  public final Command autoFiringSequence(ELauncherSpeed launcherSpeed, EShoulerPosition shoulerPosition) {
    return Commands.sequence(
        prepFireNote(launcherSpeed, shoulerPosition),
        fireNoteWithTimeout());
  }

  public final Command prepFireNote(ELauncherSpeed launcherSpeed, EShoulerPosition shoulerPosition) {
    return Commands.parallel(
        shoulder.setPositionDutyCycle(shoulerPosition),
        launcher.setVelocityVoltage(launcherSpeed));
  }

  public final Command fireNote() {
    return Commands.parallel(
        shoulder.waitUntilAtSetpoint(),
        launcher.waitUntilAtSetpoint())
        .andThen(indexer.setDutyCycle(1));
  }

  public final Command fireNoteWithTimeout() {
    return Commands.parallel(
        shoulder.waitUntilAtSetpoint(),
        launcher.waitUntilAtSetpoint())
        .andThen(indexer.setDutyCycle(1).withTimeout(0.7));
  }

  public final Command intakeNote() {
    return shoulder.setPositionDutyCycle(EShoulerPosition.HOME)
        .andThen(Commands.deadline(
            indexer.stageNote(),
            intake.setDutyCycle(1))
            .andThen(Commands.parallel(
                indexer.centerNote(),
                intake.setDutyCycle(0))));
  }

  public final Command startingConfiguration() {
    return Commands.parallel(
        shoulder.setPositionDutyCycle(EShoulerPosition.HOME),
        launcher.setVelocityVoltage(ELauncherSpeed.IDLE),
        indexer.setDutyCycle(0),
        intake.setDutyCycle(0));
  }

  public final Command lightSignals() {
    return Commands.run(() -> {
      if (indexer.isNoteStaged()) {
        systemLights.setLeds(EColorProfile.RED);
      } else if (shoulder.isNearPoint(EShoulerPosition.HOME)) {
        systemLights.setLeds(EColorProfile.BLUE);
      } else {
        systemLights.setLeds(EColorProfile.GREEN);
      }
    }, systemLights);
  }

  public final Command unjam() {
    return Commands.parallel(
        intake.setDutyCycle(-1),
        indexer.setDutyCycle(-0.1),
        launcher.setDutyCycle(-0.5));
  }

  public final Command afterJam() {
    return Commands.parallel(
        intake.setDutyCycle(0),
        indexer.setDutyCycle(0),
        launcher.setDutyCycle(0));
  }

  public final Command visionOdometryUpdate() {
    return Commands.run(
        () -> {
          commandSwerveDrivetrain.updateOmodetryWithLimelight(limelight);
        });
  }
}