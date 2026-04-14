package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.Command;

public class GoToHumanPlayer implements Command {
    private FollowPath followPath;
    private final Pose targetPose;

    public GoToHumanPlayer(boolean isBlue) {
        this.targetPose = isBlue
                ? Constants.SnapPositions.BLUE_HUMAN_PLAYER
                : Constants.SnapPositions.RED_HUMAN_PLAYER;
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