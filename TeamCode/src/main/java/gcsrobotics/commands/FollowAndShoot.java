package gcsrobotics.commands;

import com.pedropathing.geometry.Pose;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.InstantCommand;

public class FollowAndShoot implements Command {
    private final ShootingPosition position;
    private SeriesCommand sequence;

    public FollowAndShoot(ShootingPosition position) {
        this.position = position;
    }

    @Override
    public void init() {
        Pose targetPose = new Pose(
                position.poseX,
                position.poseY,
                position.poseHeading
        );

        sequence = new SeriesCommand(
                // Step 1: drive to shooting position while spinning up flywheels
                new ParallelCommand(
                        new FollowPath(targetPose),
                        new InstantCommand(() -> OpModeBase.INSTANCE.setFlywheelPower(1.0)),
                        new SetHoodAngle(position)
                ),
                // Step 2: shoot once in position
                new Shoot(position)
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