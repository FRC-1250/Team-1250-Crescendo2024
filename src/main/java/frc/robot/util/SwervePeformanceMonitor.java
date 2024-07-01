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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePeformanceMonitor {
  private Pose2d lastPose;
  private double lastTimestamp;
  private final List<CANCoderPerformanceMonitor> cancoders;
  private final List<TalonFXPerformanceMonitor> driveMotors;
  private final List<TalonFXPerformanceMonitor> steeringMotors;

  private final DoublePublisher velocityX;
  private final DoublePublisher velocityY;
  private final DoublePublisher speed;
  private final DoublePublisher odomFreq;
  private final DoubleArrayPublisher poses;
  private final StringPublisher command;
  private final StringPublisher moduleOrder;

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
      cancoders.add(new CANCoderPerformanceMonitor(swerveModules[i].getCANcoder(), subsystemName,
          String.format("module %d/sensor", i)));
      driveMotors.add(new TalonFXPerformanceMonitor(swerveModules[i].getDriveMotor(), subsystemName,
          String.format("module %d/drive", i)));
      steeringMotors.add(new TalonFXPerformanceMonitor(swerveModules[i].getDriveMotor(), subsystemName,
          String.format("module %d/steer", i)));
    }

    velocityX = NetworkTableInstance.getDefault().getTable(subsystemName).getDoubleTopic("VelocityX").publish();
    velocityY = NetworkTableInstance.getDefault().getTable(subsystemName).getDoubleTopic("VelocityY").publish();
    speed = NetworkTableInstance.getDefault().getTable(subsystemName).getDoubleTopic("Speed").publish();
    odomFreq = NetworkTableInstance.getDefault().getTable(subsystemName).getDoubleTopic("OdometryFrequency").publish();
    poses = NetworkTableInstance.getDefault().getTable(subsystemName).getDoubleArrayTopic("Pose").publish();
    command = NetworkTableInstance.getDefault().getTable(subsystemName).getStringTopic("Command").publish();
    moduleOrder = NetworkTableInstance.getDefault().getTable(subsystemName).getStringTopic("ModuleOrder").publish();
    moduleOrder.accept("FL, FR, BL, BR");

    currentStateSupplier = () -> {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < swerveModules.length; i++) {
        states[i] = swerveModules[i].getCurrentState();
      }
      return states;
    };

    currentStatesPublisher = NetworkTableInstance.getDefault()
        .getTable(subsystemName)
        .getStructArrayTopic("CurrentModuleStates", SwerveModuleState.struct).publish();

    targetStateSupplier = () -> {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < swerveModules.length; i++) {
        states[i] = swerveModules[i].getCurrentState();
      }
      return states;
    };

    targetStatesPublisher = NetworkTableInstance.getDefault()
        .getTable(subsystemName)
        .getStructArrayTopic("TargetModuleStates", SwerveModuleState.struct).publish();
  }

  public void telemeterize(SwerveDriveState state, Command currentCommand) {
    try {
      currentStatesPublisher.accept(currentStateSupplier.get());
      targetStatesPublisher.accept(targetStateSupplier.get());

      poses.set(new double[] { state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees() });

      double currentTime = Utils.getCurrentTimeSeconds();
      double diffTime = currentTime - lastTimestamp;
      lastTimestamp = currentTime;
      Translation2d distanceDiff = state.Pose.minus(lastPose).getTranslation();
      lastPose = state.Pose;

      Translation2d velocities = distanceDiff.div(diffTime);

      speed.set(velocities.getNorm());
      velocityX.set(velocities.getX());
      velocityY.set(velocities.getY());
      odomFreq.set(1.0 / state.OdometryPeriod);

      for (int i = 0; i < cancoders.size(); i++) {
        cancoders.get(i).telemeterize();
        driveMotors.get(i).telemeterize();
        steeringMotors.get(i).telemeterize();
      }

      if (currentCommand != null) {
        command.set(currentCommand.getName());
      } else {
        command.set("None");
      }
    } catch (Exception e) {
       DataLogManager.log(String.format("GatorBot: Not able to process SWERVE telemetry: %s", e.getMessage()));
    }
  }
}
