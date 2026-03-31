package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class GoToHumanPlayer implements Command {
    private FollowPath followPath;
    private final Pose targetPose;

    private static final Pose HUMAN_PLAYER_POSE_BLUE = new Pose(
            0.0,    // TODO: tune X position
            0.0,    // TODO: tune Y position
            0.0     // TODO: tune heading (radians)
    );

    private static final Pose HUMAN_PLAYER_POSE_RED = new Pose(
            0.0,    // TODO: 144 - HUMAN_PLAYER_POSE_BLUE.getX()
            0.0,    // TODO: HUMAN_PLAYER_POSE_BLUE.getY()
            0.0     // TODO: Math.PI - HUMAN_PLAYER_POSE_BLUE.getHeading()
    );

    public GoToHumanPlayer(boolean isBlue) {
        this.targetPose = isBlue ? HUMAN_PLAYER_POSE_BLUE : HUMAN_PLAYER_POSE_RED;
    }

    @Override
    public void init() {
        followPath = new FollowPath(
                OpModeBase.INSTANCE.follower.getPose(),
                targetPose
        );
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