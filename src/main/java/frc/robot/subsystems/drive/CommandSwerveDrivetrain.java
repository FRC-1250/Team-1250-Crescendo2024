package frc.robot.subsystems.drive;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.SwervePeformanceMonitor;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private boolean hasAppliedOperatorPerspective = false;
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	private final SwervePeformanceMonitor swerveMonitor;
	private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

	public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		configurePathPlanner();
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

	public ChassisSpeeds getCurrentRobotChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
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
						SwerveConfig.kSpeedAt12VoltsMps,
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

	private void checkForVisionTarget() {
		boolean doRejectUpdate = false;
		LimelightHelpers.SetRobotOrientation("limelight", m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0,
				0, 0, 0, 0);
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

	@Override
	public void periodic() {
		if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent((allianceColor) -> {
				setOperatorPerspectiveForward(
						allianceColor == Alliance.Red ? SwerveConfig.RedAlliancePerspectiveRotation
								: SwerveConfig.BlueAlliancePerspectiveRotation);
				hasAppliedOperatorPerspective = true;
			});
		}

		if (SwerveConfig.ALLOW_VISION_ODOMETRY_CORRECTION) {
			checkForVisionTarget();
		}
		swerveMonitor.telemeterize(this.getState(), this.getCurrentCommand());
	}
}