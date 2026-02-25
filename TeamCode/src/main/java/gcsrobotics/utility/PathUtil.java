package gcsrobotics.utility;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import gcsrobotics.control.OpModeBase;

public class PathUtil {
    private static final OpModeBase r = OpModeBase.INSTANCE;
    public static PathChain path(Pose... poses){
        if (poses.length >= 3) {
            return r.follower.pathBuilder().addPath(new Path(new BezierCurve(poses))).build();
        } else if (poses.length == 2) {
            return r.follower.pathBuilder().addPath(new Path(new BezierLine(poses[0], poses[1]))).build();
        } else {
            throw new IllegalArgumentException("Path must have at least 2 points");
        }
    }

    public static PathChain path(Path... paths) {
        return r.follower.pathBuilder().addPaths(paths).build();
    }
}
