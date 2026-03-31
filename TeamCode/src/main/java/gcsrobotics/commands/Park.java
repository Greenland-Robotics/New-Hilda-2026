package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

public class Park implements Command {
    private SeriesCommand sequence;
    private final Pose targetPose;

    private static final Pose PARK_POSE_BLUE = new Pose(
            0.0,    // TODO: tune X position
            0.0,    // TODO: tune Y position
            0.0     // TODO: tune heading (radians)
    );

    private static final Pose PARK_POSE_RED = new Pose(
            0.0,    // TODO: 144 - PARK_POSE_BLUE.getX()
            0.0,    // TODO: PARK_POSE_BLUE.getY()
            0.0     // TODO: Math.PI - PARK_POSE_BLUE.getHeading()
    );

    public Park(boolean isBlue) {
        this.targetPose = isBlue ? PARK_POSE_BLUE : PARK_POSE_RED;
    }

    @Override
    public void init() {
        sequence = new SeriesCommand(
                new FollowPath(
                        OpModeBase.INSTANCE.follower.getPose(),
                        targetPose
                ),
                new InstantCommand(() -> {
                    OpModeBase.INSTANCE.intakeMotor.setPower(0);
                    OpModeBase.INSTANCE.setFlywheelVelocity(0);
                    OpModeBase.INSTANCE.follower.breakFollowing();
                })
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