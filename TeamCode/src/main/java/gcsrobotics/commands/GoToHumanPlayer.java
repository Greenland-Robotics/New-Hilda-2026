package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class GoToHumanPlayer implements Command {
    private final FollowPath followPath;

    // =====================================================
    // TODO: Replace all 0.0 values with real field coordinates
    // from the Localization Test in your Tuning OpMode
    // =====================================================

    // Blue side human player zone pose
    private static final Pose HUMAN_PLAYER_POSE_BLUE = new Pose(
            0.0,    // TODO: tune X position
            0.0,    // TODO: tune Y position
            0.0     // TODO: tune heading (radians)
    );

    // Red side human player zone pose
    // Formula: redX = 144 - blueX, redY = blueY, redHeading = Math.PI - blueHeading
    private static final Pose HUMAN_PLAYER_POSE_RED = new Pose(
            0.0,    // TODO: 144 - HUMAN_PLAYER_POSE_BLUE.getX()
            0.0,    // TODO: HUMAN_PLAYER_POSE_BLUE.getY()
            0.0     // TODO: Math.PI - HUMAN_PLAYER_POSE_BLUE.getHeading()
    );

    public GoToHumanPlayer(boolean isBlue) {
        Pose targetPose = isBlue ? HUMAN_PLAYER_POSE_BLUE : HUMAN_PLAYER_POSE_RED;
        followPath = new FollowPath(
                OpModeBase.INSTANCE.follower.getPose(),
                targetPose
        );
    }

    @Override
    public void init() {
        followPath.init();
    }

    @Override
    public void loop() {
        followPath.loop();
    }

    @Override
    public boolean isFinished() {
        return followPath.isFinished();
    }
}