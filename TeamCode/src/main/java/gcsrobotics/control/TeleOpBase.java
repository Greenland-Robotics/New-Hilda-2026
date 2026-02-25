package gcsrobotics.control;

public abstract class TeleOpBase extends OpModeBase {

    protected abstract void initialize();
    protected abstract void runLoop();

    protected boolean driveMode = true;

    @Override
    protected void initInternal() {
        initialize();
        follower.startTeleOpDrive();
    }

    @Override
    protected void loopInternal() {
        follower.update();

        if (driveMode) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
        }

        runLoop();
    }
}
