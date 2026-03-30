package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class LiftJack implements Command {
    private final OpModeBase robot = OpModeBase.INSTANCE;
    private long startTime;

    // TODO: Add jack lift constants to RobotConstants and implement jack hardware
    private static final double JACK_LIFT_POWER = 1.0;
    private static final long JACK_LIFT_TIME_MS = 2000;

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        // TODO: Implement jack lifting mechanism - needs hardware definition in OpModeBase
        // robot.jackMotor.setPower(JACK_LIFT_POWER);
    }

    @Override
    public void loop() {
        // Motor is running, nothing to update each loop
    }

    @Override
    public boolean isFinished() {
        boolean timeExpired = System.currentTimeMillis() - startTime >= JACK_LIFT_TIME_MS;
        if (timeExpired) {
            // TODO: Stop jack motor when time expired
            // robot.jackMotor.setPower(0);
        }
        return timeExpired;
    }
}
