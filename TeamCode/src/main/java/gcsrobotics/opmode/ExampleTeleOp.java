package gcsrobotics.opmode;

import gcsrobotics.control.TeleOpBase;

public class ExampleTeleOp extends TeleOpBase {
    @Override
    protected void initialize() {
        INSTANCE = this;
        follower.startTeleOpDrive();
    }

    @Override
    protected void runLoop() {
    }
}
