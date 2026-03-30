package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.InstantCommand;

public class Park implements Command {
    private final SeriesCommand sequence;

    // =====================================================
    // TODO: Replace all 0.0 values with real field coordinates
    // from the Localization Test in your Tuning OpMode
    // =====================================================

    // Blue side park pose
    private static final Pose PARK_POSE_BLUE = new Pose(
            0.0,    // TODO: tune X position
            0.0,    // TODO: tune Y position
            0.0     // TODO: tune heading (radians)
    );

    // Red side park pose
    private static final Pose PARK_POSE_RED = new Pose(
            0.0,    // TODO: 144 - PARK_POSE_BLUE.getX()
            0.0,    // TODO: PARK_POSE_BLUE.getY()
            0.0     // TODO: Math.PI - PARK_POSE_BLUE.getHeading()
    );

    public Park(boolean isBlue) {
        Pose targetPose = isBlue ? PARK_POSE_BLUE : PARK_POSE_RED;

        sequence = new SeriesCommand(
                new FollowPath(
                        OpModeBase.INSTANCE.follower.getPose(),
                        targetPose
                ),
                new InstantCommand(() -> {
                    OpModeBase.INSTANCE.intakeMotor.setPower(0);
                    OpModeBase.INSTANCE.setFlywheelPower(0);
                    OpModeBase.INSTANCE.follower.bre