package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HolonomicPaths {
    public final HolonomicPathComponents speakerCenterAndLeaveStartingZone;
    public final HolonomicPathComponents speakerAmpSideAndLeaveStartingZone;
    public final HolonomicPathComponents speakerSourceSideAndLeaveStartingZone;

    public HolonomicPaths() {
        speakerCenterAndLeaveStartingZone = SpeakerCenterAndLeaveStartingZone();
        speakerAmpSideAndLeaveStartingZone = SpeakerAmpSideAndLeaveStartingZone();
        speakerSourceSideAndLeaveStartingZone = SpeakerSourceSideAndLeaveStartingZone();
    }

    private HolonomicPathComponents SpeakerCenterAndLeaveStartingZone() {
        Rotation2d startingRotation = PathMetadata.SPEAKER_CENTER_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        List<RotationTarget> rotationTargets = new ArrayList<>();
        GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));

        poses.add(new Pose2d(PathMetadata.SPEAKER_CENTER, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_SPEAKER_CENTER, Rotation2d.fromDegrees(0)));
        return new HolonomicPathComponents(startingRotation, poses, rotationTargets, goalEndState);
    }

    private HolonomicPathComponents SpeakerAmpSideAndLeaveStartingZone() {
        Rotation2d startingRotation = PathMetadata.SPEAKER_AMP_SIDE_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        List<RotationTarget> rotationTargets = new ArrayList<>();
        GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));

        poses.add(new Pose2d(PathMetadata.SPEAKER_AMP_SIDE, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_AMP_SIDE, Rotation2d.fromDegrees(0)));
        return new HolonomicPathComponents(startingRotation, poses, rotationTargets, goalEndState);
    }

    private HolonomicPathComponents SpeakerSourceSideAndLeaveStartingZone() {
        Rotation2d startingRotation = PathMetadata.SPEAKER_SOURCE_SIDE_APPROACH_ANGLE;
        List<Pose2d> poses = new ArrayList<>();
        List<RotationTarget> rotationTargets = new ArrayList<>();
        GoalEndState goalEndState = new GoalEndState(0, Rotation2d.fromDegrees(0));

        poses.add(new Pose2d(PathMetadata.SPEAKER_SOURCE_SIDE, Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(PathMetadata.NOTE_PODIUM, Rotation2d.fromDegrees(0)));
        return new HolonomicPathComponents(startingRotation, poses, rotationTargets, goalEndState);
    }
}