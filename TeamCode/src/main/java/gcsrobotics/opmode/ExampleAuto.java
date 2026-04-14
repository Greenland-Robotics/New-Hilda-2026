package gcsrobotics.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import gcsrobotics.control.AutoBase;

public class ExampleAuto extends AutoBase {



    @Override
    protected void buildCommands() {

    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void runLoop() {

    }

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(38.993, 40.015),
                                    new Pose(14.112, 34.297)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(14.112, 34.297),
                                    new Pose(21.741, 55.175)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.741, 55.175),
                                    new Pose(48.636, 79.462),
                                    new Pose(18.401, 80.465)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
