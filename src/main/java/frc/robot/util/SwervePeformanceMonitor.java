package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePeformanceMonitor {
  private Pose2d lastPose;
  private double lastTimestamp;
  private final List<CANCoderPerformanceMonitor> cancoders;
  private final List<TalonFXPerformanceMonitor> driveMotors;
  private final List<TalonFXPerformanceMonitor> steeringMotors;

  private final Supplier<SwerveModuleState[]> currentStateSupplier;
  private final Supplier<SwerveModuleState[]> targetStateSupplier;
  private final StructArrayPublisher<SwerveModuleState> currentStatesPublisher;
  private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;

  public SwervePeformanceMonitor(String subsystemName, SwerveModule[] swerveModules) {
    lastPose = new Pose2d();
    lastTimestamp = Utils.getCurrentTimeSeconds();
    cancoders = new ArrayList<>();
    driveMotors = new ArrayList<>();
    steeringMotors = new ArrayList<>();

    for (int i = 0; i < swerveModules.length; i++) {
      cancoders.add(new CANCoderPerformanceMonitor(swerveModules[i].getCANcoder(),
       String.format("%s/Module%d", subsystemName, i)));
      driveMotors.add(new TalonFXPerformanceMonitor(swerveModules[i].getDriveMotor(), subsystemName,
          String.format("Module%d/Drive", i)));
      steeringMotors.add(new TalonFXPerformanceMonitor(swerveModules[i].getDriveMotor(), subsystemName,
          String.format("Module%d/Steer", i)));
    }

    SmartDashboard.putString(String.format("%s/ModuleOrder", subsystemName), "FL, FR, BL, BR");

    currentStateSupplier = () -> {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < swerveModules.length; i++) {
        states[i] = swerveModules[i].getCurrentState();
      }
      return states;
    };

    currentStatesPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard").getSubTable(subsystemName)
        .getStructArrayTopic("CurrentModuleStates", SwerveModuleState.struct).publish();

    targetStateSupplier = () -> {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < swerveModules.length; i++) {
        states[i] = swerveModules[i].getCurrentState();
      }
      return states;
    };

    targetStatesPublisher = NetworkTableInstance.getDefault()
        .getTable("SmartDashboard").getSubTable(subsystemName)
        .getStructArrayTopic("TargetModuleStates", SwerveModuleState.struct).publish();
  }

  public void telemeterize(Supplier<SwerveDriveState> sds, Supplier<Command> c) {
    telemeterize(sds.get(), c.get());
  }

  public void telemeterize(SwerveDriveState state, Command currentCommand) {
    currentStatesPublisher.accept(currentStateSupplier.get());
    targetStatesPublisher.accept(targetStateSupplier.get());

    SmartDashboard.putNumberArray("Swerve/Pose", new double[] { state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees() });
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTimestamp;
    lastTimestamp = currentTime;
    Translation2d distanceDiff = state.Pose.minus(lastPose).getTranslation();
    lastPose = state.Pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    SmartDashboard.putNumber("Swerve/Speed", velocities.getNorm());
    SmartDashboard.putNumber("Swerve/VelocityX", velocities.getX());
    SmartDashboard.putNumber("Swerve/VelocityY", velocities.getY());
    SmartDashboard.putNumber("Swerve/OdomFreq", 1.0 / state.OdometryPeriod);

    for (int i = 0; i < cancoders.size(); i++) {
      cancoders.get(i).push();
      driveMotors.get(i).push();
      steeringMotors.get(i).push();
    }
  }
}
