package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.TunerConstants;
import frc.robot.util.SwervePeformanceMonitor;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    SwervePeformanceMonitor swerveMonitor;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        configureCurrentLimits();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        swerveMonitor = new SwervePeformanceMonitor("Swerve", Modules);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        configureCurrentLimits();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        swerveMonitor = new SwervePeformanceMonitor("Swerve", Modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).withName("Drive");
    }

     public Command applyRequestWithName(Supplier<SwerveRequest> requestSupplier, String name) {
        return applyRequest(requestSupplier).withName(name);
    }

    private void configureCurrentLimits() {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();

        for (int i = 0; i < Modules.length; i++) {
            var driveConfigurator = Modules[i].getDriveMotor().getConfigurator();
            driveConfigurator.refresh(openLoopRampsConfigs);
            currentLimitsConfigs.SupplyCurrentLimitEnable = true;
            currentLimitsConfigs.SupplyCurrentLimit = 30;
            driveConfigurator.apply(openLoopRampsConfigs);

            driveConfigurator.refresh(openLoopRampsConfigs);
            openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.25;
            driveConfigurator.apply(openLoopRampsConfigs);

            driveConfigurator.refresh(closedLoopRampsConfigs);
            closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.25;
            driveConfigurator.apply(closedLoopRampsConfigs);

            var steerConfigurator = Modules[i].getSteerMotor().getConfigurator();
            steerConfigurator.refresh(currentLimitsConfigs);
            currentLimitsConfigs.SupplyCurrentLimitEnable = true;
            currentLimitsConfigs.SupplyCurrentLimit = 30;
            steerConfigurator.apply(currentLimitsConfigs);
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(4, 0, 0),
                        new PIDConstants(3, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setOdometry(Rotation2d angle, Pose2d location) {
        m_yawGetter.waitForUpdate(1);
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValueAsDouble() - angle.getDegrees()),
                m_modulePositions, location);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public double getHeading() {
        return m_odometry.getEstimatedPosition().getRotation().getDegrees();
    }

    @Override
    public void periodic() {
      swerveMonitor.telemeterize(this.getState(), this.getCurrentCommand());
    }
}
