package gcsrobotics.commands;

import gcsrobotics.vertices.Command;

public class ExampleCommand implements Command {

    public void init(){
        // Code to run at the start of command running
    }

    public void loop(){
        // Code to run while the command is running
    }

    public boolean isFinished(){
        return true; // replace `true` with the condition for when the command should end
    }
}
