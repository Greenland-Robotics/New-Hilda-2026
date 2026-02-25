package gcsrobotics.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

/// ======== Example Command ========
public class Shoot implements Command {
    OpModeBase robot = OpModeBase.INSTANCE;
    private final ElapsedTime shotTimer = new ElapsedTime();

    private final static long SHOT_TIME_MS = 1000;
    private final static int FLYWHEEL_SPEED = 1000;


    public void init() {
        shotTimer.reset();
    }

    public void loop() {
        robot.flywheel.setVelocity(FLYWHEEL_SPEED);

        if (shotTimer.milliseconds() >= SHOT_TIME_MS) robot.flywheel.setVelocity(0);
    }

    public boolean isFinished() {return shotTimer.milliseconds() >= SHOT_TIME_MS;}
}
