package gcsrobotics.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.commands.FollowPath;
import gcsrobotics.control.TeleOpBase;
import gcsrobotics.vertices.ButtonAction;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

@TeleOp(name="Example TeleOp")
@Disabled
public class ExampleTeleOp extends TeleOpBase {
    private boolean driveMode = true;
    private ButtonAction pathToCenter;

    @Override
    protected void initialize() {
        INSTANCE = this;
        follower.startTeleOpDrive();

        pathToCenter = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> driveMode = false),
                        new FollowPath(
                                new Pose(getX(), getY(), getHeading()),
                                new Pose(72, 72, 0)
                        )
                ),commandRunner
        );
    }

    @Override
    protected void runLoop() {
        pathToCenter.update(gamepad1.a);

        if (driveMode) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }
    }
}
