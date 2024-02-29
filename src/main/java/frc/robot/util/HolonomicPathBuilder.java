package frc.robot.util;

import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TunerConstants;

public class HolonomicPathBuilder {
    private static final double Y_COORDINATE_MIDPOINT = Units.feetToMeters(13.5);

    public static PathPlannerPath build(Alliance alliance, HolonomicPathComponents holonomicPathComponents) {
        if (alliance == Alliance.Red) {
            holonomicPathComponents = flipPathToRedAlliance(holonomicPathComponents);
        }
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.bezierFromPoses(holonomicPathComponents.getPathPoints()),
                holonomicPathComponents.getRotationTargets(),
                holonomicPathComponents.getConstraintsZones(),
                Collections.emptyList(),
                TunerConstants.MAX_PATH_CONSTRAINTS,
                holonomicPathComponents.getGoalEndState(),
                false,
                holonomicPathComponents.getStartingRotation());

        path.preventFlipping = true;
        return path;
    }

    private static HolonomicPathComponents flipPathToRedAlliance(HolonomicPathComponents holonomicPathComponents) {
        List<Pose2d> poses = holonomicPathComponents.getPathPoints();
        List<RotationTarget> rotationTargets = holonomicPathComponents.getRotationTargets();
        GoalEndState goalEndState = holonomicPathComponents.getGoalEndState();
        Rotation2d startingRotation = holonomicPathComponents.getStartingRotation();

        for (int i = 0; i < poses.size(); i++) {
            poses.set(i, new Pose2d(reflectOverY(poses.get(i).getTranslation()),
                    Rotation2d.fromDegrees(-poses.get(i).getRotation().getDegrees())));
        }

        for (int i = 0; i < rotationTargets.size(); i++) {
            rotationTargets.set(i, new RotationTarget(rotationTargets.get(i).getPosition(),
                    Rotation2d.fromDegrees(-rotationTargets.get(i).getTarget().getDegrees())));
        }

        goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(-goalEndState.getRotation().getDegrees()));
        startingRotation = Rotation2d.fromDegrees(-startingRotation.getDegrees());
        return new HolonomicPathComponents(
                startingRotation,
                poses,
                rotationTargets,
                holonomicPathComponents.getConstraintsZones(),
                goalEndState);
    }

    private static Translation2d reflectOverY(Translation2d point) {
        return new Translation2d(point.getX(), ((Y_COORDINATE_MIDPOINT - point.getY()) + Y_COORDINATE_MIDPOINT));
    }
}