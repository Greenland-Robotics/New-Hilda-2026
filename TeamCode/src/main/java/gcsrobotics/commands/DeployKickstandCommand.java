package gcsrobotics.commands;

import gcsrobotics.vertices.Command;
import gcsrobotics.pedroPathing.Constants;

public class DeployKickstandCommand implements Command {

    private final KickstandSubsystem kickstand;
    private long startTime;

    public DeployKickstandCommand(KickstandSubsystem kickstand) {
        this.kickstand = kickstand;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        kickstand.deploy();
    }

    @Override
    public void loop() {
        // motor is already running, nothing to update each loop
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= Constants.Kickstand.DEPLOY_TIME_MS;
    }
}