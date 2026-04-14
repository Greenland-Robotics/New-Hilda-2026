package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

public class Park implements Command {
    private SeriesCommand sequence;
    private final Pose targetPose;

    public Park(boolean isBlue) {
        this.targetPose = isBlue
                ? Constants.SnapPositions.BLUE_PARK
                : Constants.SnapPositions.RED_PARK;
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