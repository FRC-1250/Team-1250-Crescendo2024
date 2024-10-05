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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
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
    private boolean allowVisionOdometryCorrection = false;

     /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
     private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
     /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
     private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
     /* Keep track if we've ever applied the operator perspective before or not */
     private boolean hasAppliedOperatorPerspective = false; 

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
        this.registerTelemetry(m_telemetryFunction);
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

    public void checkForVisionTarget() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(m_pigeon2.getRate()) > 720) {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_odometry.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }

    public void telemeterize() {
        swerveMonitor.telemeterize(this.getState(), this.getCurrentCommand());
    }

    @Override
    public void periodic() {
         if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        if (allowVisionOdometryCorrection) {
            checkForVisionTarget();
        }
    }
}
