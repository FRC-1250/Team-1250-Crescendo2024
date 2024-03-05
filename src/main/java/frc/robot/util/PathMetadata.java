package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathMetadata {
        public static final Translation2d SPEAKER_CENTER = new Translation2d(1.33, 5.55);
        public static final Translation2d SPEAKER_SOURCE_SIDE = new Translation2d(0.72, 4.40);
        public static final Translation2d SPEAKER_AMP_SIDE = new Translation2d(0.72, 6.69);
        public static final Translation2d AMP = new Translation2d(1.82, 7.65);
        public static final Translation2d NOTE_AMP_SIDE = new Translation2d(2.90, 7.1);
        public static final Translation2d NOTE_SPEAKER_CENTER = new Translation2d(2.90, 5.55);
        public static final Translation2d NOTE_PODIUM = new Translation2d(2.90, 4.10);

        // Aligned with a landmark on the X axis
        public static final Translation2d NOTE_CENTERLINE_AMP_ALIGNED = new Translation2d(8.30, 7.45);
        public static final Translation2d NOTE_CENTERLINE_SPEAKER_ALIGNED = new Translation2d(8.30, 5.75);
        public static final Translation2d NOTE_CENTERLINE_PODIUM_ALIGNED = new Translation2d(8.30, 4.10);
        public static final Translation2d NOTE_CENTERLINE_NO_ALIGNMENT = new Translation2d(8.30, 2.45);
        public static final Translation2d NOTE_CENTERLINE_SOURCE_ALIGNED = new Translation2d(8.30, 0.78);

        // Poses for starting and end conditions
        public static final Rotation2d SPEAKER_CENTER_APPROACH_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d SPEAKER_SOURCE_SIDE_APPROACH_ANGLE = Rotation2d.fromDegrees(-60);
        public static final Rotation2d SPEAKER_AMP_SIDE_APPROACH_ANGLE = Rotation2d.fromDegrees(60);
        public static final Rotation2d AMP_APPROACH_ANGLE = Rotation2d.fromDegrees(90);
}