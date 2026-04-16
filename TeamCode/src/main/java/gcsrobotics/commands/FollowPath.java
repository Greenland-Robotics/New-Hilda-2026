package gcsrobotics.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class FollowPath implements Command {
    private final OpModeBase r = OpModeBase.INSTANCE;
    private final PathChain path;
    private final boolean holdEnd;

    // ── Standard constructor — does not hold end ──
    public FollowPath(PathChain path) {
        this.path = path;
        this.holdEnd = false;
    }

    // ── holdEnd constructor — pass true to hold position at endpoint ──
    public FollowPath(PathChain path, boolean holdEnd) {
        this.path = path;
        this.holdEnd = holdEnd;
    }

    public FollowPath(Path path) {
        this.path = r.follower.pathBuilder().addPath(path).build();
        this.holdEnd = false;
    }

    // ── Pose varargs constructor ──
    // 2 poses: straight line with linear heading interpolation from pose[0] to pose[1]
    // 3+ poses: bezier curve (no heading interpolation — set manually if needed)
    public FollowPath(Pose... poses) {
        if (poses.length >= 3) {
            this.path = r.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(poses)))
                    .build();
        } else if (poses.length == 2) {
            this.path = r.follower.pathBuilder()
                    .addPath(new Path(new BezierLine(poses[0], poses[1])))
                    .setLinearHeadingInterpolation(
                            poses[0].getHeading(),
                            poses[1].getHeading())
                    .build();
        } else {
            throw new IllegalArgumentException("Path must have at least 2 points");
        }
        this.holdEnd = false;
    }

    @Override
    public void init() {
        r.follower.followPath(path, holdEnd);
    }

    @Override
    public void loop() {
        r.follower.update();
    }

    @Override
    public boolean isFinished() {
        return !r.follower.isBusy();
    }
}