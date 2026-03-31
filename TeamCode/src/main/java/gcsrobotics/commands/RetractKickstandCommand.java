package gcsrobotics.commands;

import gcsrobotics.vertices.Command;

public class RetractKickstandCommand implements Command {

    private final KickstandSubsystem kickstand;

    public RetractKickstandCommand(KickstandSubsystem kickstand) {
        this.kickstand = kickstand;
    }

    @Override
    public void init() {
        kickstand.retract();
    }

    @Override
    public void loop() {
        // servo runs continuously while button held — nothing to update
    }

    @Override
    public boolean isFinished() {
        // never self-terminates — ButtonAction cancels this when button is released
        return false;
    }
}