package gcsrobotics.opmode;

import com.pedropathing.geometry.Pose;
import static gcsrobotics.utility.PathUtil.*;
import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.Shoot;
import gcsrobotics.control.AutoBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;

public class ExampleAuto extends AutoBase {
    private Pose startPose, shootPose, intakePose;
    private Command shootPreload, intakeLine1;
    protected void buildCommands() {
        shootPreload = new SeriesCommand(
                new FollowPath(path(startPose, shootPose)),
                new Shoot()
        );

        intakeLine1 = new ParallelCommand(
                new FollowPath(path(shootPose, intakePose)),
                new InstantCommand(() -> intake.setVelocity(1000))
        );

    }

    @Override
    protected void initialize() {
        INSTANCE = this;
        commandRunner = new CommandRunner(new SeriesCommand(
                shootPreload,
                intakeLine1
        ));
    }

    @Override
    protected void runLoop() {
        telemetry.addData("x pos", follower.getPose().getX());
        telemetry.addData("y pos", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }
}
