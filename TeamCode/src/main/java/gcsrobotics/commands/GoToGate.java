package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.SeriesCommand;

public class GoToGate implements Command {
    private SeriesCommand sequence;
    private final Pose targetPose;

    private static final Pose GATE_POSE_BLUE = new Pose(
            0.0,    // TODO: tune X position
            0.0,    // TODO: tune Y position
            0.0     // TODO: tune heading (radians)
    );

    private static final Pose GATE_POSE_RED = new Pose(
            0.0,    // TODO: 144 - GATE_POSE_BLUE.getX()
            0.0,    // TODO: GATE_POSE_BLUE.getY()
            0.0     // TODO: Math.PI - GATE_POSE_BLUE.getHeading()
    );

    public GoToGate(boolean isBlue) {
        this.targetPose = isBlue ? GATE_POSE_BLUE : GATE_POSE_RED;
    }

    @Override
    public void init() {
        sequence = new SeriesCommand(
                new FollowPath(
                        OpModeBase.INSTANCE.follower.getPose(),
                        targetPose
                ),
                new OpenGate()
        );
        sequence.init();
    }

    @Override
    public void loop() {
        sequence.loop();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }
}