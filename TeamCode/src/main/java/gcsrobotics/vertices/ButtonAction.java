package gcsrobotics.vertices;

import java.util.function.Supplier;

public class ButtonAction {
    public boolean lastPressed = false;

    private final Supplier<Command> commandFactory;
    private final CommandRunner commandRunner;

    public ButtonAction(Supplier<Command> commandFactory, CommandRunner commandRunner) {
        this.commandFactory = commandFactory;
        this.commandRunner = commandRunner;
    }

    public void update(boolean input) {
        if (input && !lastPressed) {
            lastPressed = true;
            commandRunner.run(commandFactory.get());
        } else if (!input) {
            lastPressed = false;
        }
    }
}