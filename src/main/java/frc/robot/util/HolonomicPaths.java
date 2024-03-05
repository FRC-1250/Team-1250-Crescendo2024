package frc.robot.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TunerConstants;

public class HolonomicPaths {

    public static PathPlannerPath speakerCenter(Alliance alliance) {
        Rotation2d startingRotation = PathMetadata.SPEAKER_CENTER_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        poses.add(new Pose2d(PathMetadata.SPEAKER_CENTER, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_SPEAKER_CENTER.plus(new Translation2d(.1,0)), Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.SPEAKER_CENTER, Rotation2d.fromDegrees(180)));

        List<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(new ConstraintsZone(0.5, 1.2, TunerConstants.LOW_PATH_CONSTRAINTS));

        GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));
        return HolonomicPathBuilder.build(
                alliance,
                new HolonomicPathComponents(
                        startingRotation,
                        poses,
                        Collections.emptyList(),
                        constraintsZones,
                        goalEndState));
    }

    public static PathPlannerPath speakerAmpSide(Alliance alliance) {
        Rotation2d startingRotation = PathMetadata.SPEAKER_AMP_SIDE_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        poses.add(new Pose2d(PathMetadata.SPEAKER_AMP_SIDE, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_AMP_SIDE, Rotation2d.fromDegrees(45)));
        poses.add(new Pose2d(PathMetadata.SPEAKER_AMP_SIDE, Rotation2d.fromDegrees(180)));

        List<RotationTarget> rotationTargets = new ArrayList<>();
        rotationTargets.add(new RotationTarget(0.7, Rotation2d.fromDegrees(30), false));

        List<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(new ConstraintsZone(0.5, 1.2, TunerConstants.LOW_PATH_CONSTRAINTS));

        GoalEndState goalEndState = new GoalEndState(0, PathMetadata.SPEAKER_AMP_SIDE_APPROACH_ANGLE);
        return HolonomicPathBuilder.build(
                alliance,
                new HolonomicPathComponents(
                        startingRotation,
                        poses,
                        rotationTargets,
                        constraintsZones,
                        goalEndState));
    }

       public static PathPlannerPath speakerAmpSideToAmp(Alliance alliance) {
        Rotation2d startingRotation = PathMetadata.SPEAKER_AMP_SIDE_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        poses.add(new Pose2d(PathMetadata.SPEAKER_AMP_SIDE, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_AMP_SIDE, Rotation2d.fromDegrees(-30)));
        poses.add(new Pose2d(PathMetadata.AMP, Rotation2d.fromDegrees(90)));

        List<RotationTarget> rotationTargets = new ArrayList<>();
        rotationTargets.add(new RotationTarget(0.7, Rotation2d.fromDegrees(-30), false));

        List<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(new ConstraintsZone(0.8, 1.2, TunerConstants.LOW_PATH_CONSTRAINTS));

        GoalEndState goalEndState = new GoalEndState(0, PathMetadata.AMP_APPROACH_ANGLE);
        return HolonomicPathBuilder.build(
                alliance,
                new HolonomicPathComponents(
                        startingRotation,
                        poses,
                        rotationTargets,
                        constraintsZones,
                        goalEndState));
    }

    public static PathPlannerPath speakerSourceSide(Alliance alliance) {
        Rotation2d startingRotation = PathMetadata.SPEAKER_SOURCE_SIDE_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        poses.add(new Pose2d(PathMetadata.SPEAKER_SOURCE_SIDE, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_PODIUM.minus(new Translation2d(0.2, 0)), Rotation2d.fromDegrees(0)));

        List<RotationTarget> rotationTargets = new ArrayList<>();
        rotationTargets.add(new RotationTarget(0.7, Rotation2d.fromDegrees(0), false));

        List<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(new ConstraintsZone(0.8, 1.2, TunerConstants.LOW_PATH_CONSTRAINTS));

        GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));

        return HolonomicPathBuilder.build(
                alliance,
                new HolonomicPathComponents(
                        startingRotation,
                        poses,
                        rotationTargets,
                        constraintsZones,
                        goalEndState));
    }

    public static PathPlannerPath SourceEscapePlan(Alliance alliance) { 
        Rotation2d startingRotaion = PathMetadata.SPEAKER_SOURCE_SIDE_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        poses.add(new Pose2d(PathMetadata.SPEAKER_SOURCE_SIDE, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(1.28, 1.78, Rotation2d.fromDegrees(-90)));
        poses.add(new Pose2d(2.92, .64, Rotation2d.fromDegrees(0)));

        GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));

        return HolonomicPathBuilder.build(
                alliance,
                 new HolonomicPathComponents(
                        startingRotaion, 
                        poses,
                        Collections.emptyList(),
                        Collections.emptyList(),
                        goalEndState
                 ));
    }
}