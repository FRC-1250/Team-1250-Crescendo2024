package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot speeds for general checking */
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();

    private final DoublePublisher poseX = driveStats.getDoubleTopic("Pose X").publish();
    private final DoublePublisher poseY = driveStats.getDoubleTopic("Pose Y").publish();
    private final DoublePublisher poseHeading = driveStats.getDoubleTopic("Pose Heading").publish();

    private final DoublePublisher moduleOneSpeed = driveStats.getDoubleTopic("Module 1 speed").publish();
    private final DoublePublisher moduleOneHeading = driveStats.getDoubleTopic("Module 1 heading").publish();
    private final DoublePublisher moduleTwoSpeed = driveStats.getDoubleTopic("Module 2 speed").publish();
    private final DoublePublisher moduleTwoHeading = driveStats.getDoubleTopic("Module 2 heading").publish();
    private final DoublePublisher moduleThreeSpeed = driveStats.getDoubleTopic("Module 3 speed").publish();
    private final DoublePublisher moduleThreeHeading = driveStats.getDoubleTopic("Module 3 heading").publish();
    private final DoublePublisher moduleFourSpeed = driveStats.getDoubleTopic("Module 4 speed").publish();
    private final DoublePublisher moduleFourHeading = driveStats.getDoubleTopic("Module 4 heading").publish();

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();
    private int aliveCounter = 0;

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        poseX.set(pose.getX());
        poseY.set(pose.getY());
        poseHeading.set(pose.getRotation().getDegrees());

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomPeriod.set(state.OdometryPeriod);

        /* Telemeterize the module's states */
        moduleOneSpeed.set(state.ModuleStates[0].speedMetersPerSecond);
        moduleTwoSpeed.set(state.ModuleStates[1].speedMetersPerSecond);
        moduleThreeSpeed.set(state.ModuleStates[2].speedMetersPerSecond);
        moduleFourSpeed.set(state.ModuleStates[3].speedMetersPerSecond);

        moduleOneHeading.set(state.ModuleStates[0].angle.getDegrees());
        moduleTwoHeading.set(state.ModuleStates[1].angle.getDegrees());
        moduleThreeHeading.set(state.ModuleStates[2].angle.getDegrees());
        moduleFourHeading.set(state.ModuleStates[3].angle.getDegrees());

        SmartDashboard.putNumber("Odometry Thread Alive", aliveCounter++);
    }
}
