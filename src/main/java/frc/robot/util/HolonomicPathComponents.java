package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicPathComponents {
    private Rotation2d startingRotation = new Rotation2d();
    private List<Pose2d> pathPoints = new ArrayList<>();
    private List<RotationTarget> rotationTargets = new ArrayList<>();
    private List<ConstraintsZone> constraintsZones = new ArrayList<>();
    private GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));

    public HolonomicPathComponents(Rotation2d startingRotation,
            List<Pose2d> pathPoints,
            List<RotationTarget> rotationTargets,
            List<ConstraintsZone> constraintsZones,
            GoalEndState goalEndState) {
        this.startingRotation = startingRotation;
        this.pathPoints = pathPoints;
        this.rotationTargets = rotationTargets;
        this.constraintsZones = constraintsZones;
        this.goalEndState = goalEndState;
    }

    public HolonomicPathComponents(HolonomicPathComponents holonomicPathComponents) {
        this(holonomicPathComponents.startingRotation,
                holonomicPathComponents.pathPoints,
                holonomicPathComponents.rotationTargets,
                holonomicPathComponents.constraintsZones,
                holonomicPathComponents.goalEndState);
    }

    public Rotation2d getStartingRotation() {
        return startingRotation;
    }

    public List<Pose2d> getPathPoints() {
        return pathPoints;
    }

    public List<RotationTarget> getRotationTargets() {
        return rotationTargets;
    }

    public List<ConstraintsZone> getConstraintsZones() {
        return constraintsZones;
    }

    public GoalEndState getGoalEndState() {
        return goalEndState;
    }
}