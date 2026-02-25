package gcsrobotics.control;

import gcsrobotics.vertices.CommandRunner;

public abstract class TeleOpBase extends OpModeBase {

    protected abstract void initialize();
    protected abstract void runLoop();

    protected final CommandRunner commandRunner = new CommandRunner();

    @Override
    protected void initInternal() {
        initialize();
        follower.startTeleOpDrive();
    }

    @Override
    protected void loopInternal() {
        follower.update();
        commandRunner.update();
        runLoop();
    }
}
