package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class PathToFarZone implements Command {
    private FollowPath followPath;
    private final Pose targetPose;

    private static final double FAR_ZONE_X   = 0.0;      // TODO: tune X position
    private static final double FAR_ZONE_Y   = 0.0;      // TODO: tune Y position
    private static final double HEADING_BLUE = 0.0;      // TODO: tune blue heading (radians)
    private static final double HEADING_RED  = Math.PI;  // TODO: tune red heading (radians)

    public PathToFarZone(boolean isBlue) {
        double heading = isBlue ? HEADING_BLUE : HEADING_RED;
        this.targetPose = new Pose(FAR_ZONE_X, FAR_ZONE_Y, heading);
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