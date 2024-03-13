package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        configureOpenLoopRampRates();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        configureOpenLoopRampRates();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).withName("Drive");
    }

     public Command applyRequestWithName(Supplier<SwerveRequest> requestSupplier, String name) {
        return applyRequest(requestSupplier).withName(name);
    }

    private void configureOpenLoopRampRates() {
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.1;
        openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.1;
        openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.1;

        for (int i = 0; i < Modules.length; i++) {
            Modules[i].getDriveMotor().getConfigurator().apply(openLoopRampsConfigs);
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
                () -> false, // Change this if the path needs to be flipped on red vs blue
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

    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    private void telemetry(SwerveDriveState state) {
        Pose2d pose = state.Pose;

        SmartDashboard.putNumber("Drive/Pose X", pose.getX());
        SmartDashboard.putNumber("Drive/Pose Y", pose.getY());
        SmartDashboard.putNumber("Drive/Pose Heading", pose.getRotation().getDegrees());

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        SmartDashboard.putNumber("Drive/Speed", velocities.getNorm());
        SmartDashboard.putNumber("Drive/Velocity X", velocities.getX());
        SmartDashboard.putNumber("Drive/Velocity Y", velocities.getY());
        SmartDashboard.putNumber("Drive/Odometry Period", state.OdometryPeriod);

        /* Telemeterize the module's states */
        SmartDashboard.putNumber("Drive/Module one/Speed", state.ModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Drive/Module two/Speed", state.ModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Drive/Module three/Speed", state.ModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Drive/Module four/Speed", state.ModuleStates[3].speedMetersPerSecond);

        SmartDashboard.putNumber("Drive/Module one/Heading", state.ModuleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Drive/Module two/Heading", state.ModuleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Drive/Module three/Heading", state.ModuleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Drive/Module four/Heading", state.ModuleStates[0].angle.getDegrees());

        var moduleOne = getModule(0);
        var moduleTwo = getModule(1);
        var moduleThree = getModule(2);
        var moduleFour = getModule(3);

        SmartDashboard.putNumber("Drive/Module one/Drive stator curret",
                moduleOne.getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Module one/Drive RPM",
                moduleOne.getDriveMotor().getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Drive/Module one/Steer stator curret",
                moduleOne.getSteerMotor().getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Drive/Module two/Drive stator curret",
                moduleTwo.getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Module two/Drive RPM",
                moduleTwo.getDriveMotor().getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Drive/Module two/Steer stator curret",
                moduleTwo.getSteerMotor().getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Drive/Module three/Drive stator curret",
                moduleThree.getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Module three/Drive RPM",
                moduleThree.getDriveMotor().getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Drive/Module three/Steer stator curret",
                moduleThree.getSteerMotor().getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Drive/Module four/Drive stator curret",
                moduleFour.getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Drive/Module four/Drive RPM",
                moduleFour.getDriveMotor().getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Drive/Module four/Steer stator curret",
                moduleFour.getSteerMotor().getStatorCurrent().getValueAsDouble());

        if (this.getCurrentCommand() != null) {
            SmartDashboard.putString("Drive/Command", this.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("Drive/Command", "None");
        }
    }

    @Override
    public void periodic() {
        telemetry(this.getState());
    }
}
