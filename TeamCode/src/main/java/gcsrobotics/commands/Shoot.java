package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.AwaitCommand;
import gcsrobotics.vertices.SleepCommand;
import gcsrobotics.pedroPathing.Constants;

public class Shoot implements Command {
    private final OpModeBase robot = OpModeBase.INSTANCE;
    private final ShootingPosition position;
    private SeriesCommand sequence;

    public Shoot(ShootingPosition position) {
        this.position = position;
    }

    @Override
    public void init() {
        sequence = new SeriesCommand(
                // Step 1: spin up flywheels and set hood angle simultaneously
                new InstantCommand(() -> {
                    robot.setFlywheelPower(1.0);
                    robot.hoodServo.setPosition(position.hoodPosition);
                }),
                // Step 2: wait for flywheels to reach target RPM
                new AwaitCommand(
                        () -> robot.getFlywheelVelocity() >= position.targetRPM * Constants.Shooter.RPM_THRESHOLD,
                        Constants.Shooter.FLYWHEEL_TIMEOUT_MS // timeout if RPM never reached
                ),
                // Step 3: open gate
                new InstantCommand(() -> {
                    robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
                }),
                // Step 4: wait 150ms
                new SleepCommand(150),
                // Step 5: start intake at full power
                new InstantCommand(() -> {
                    robot.intakeMotor.setPower(Constants.Intake.FORWARD_POWER);
                }),
                // Step 6: wait for ball to pass through
                new SleepCommand(Constants.Shooter.SHOOT_DURATION_MS),
                // Step 7: close gate and stop intake
                new InstantCommand(() -> {
                    robot.gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
                    robot.intakeMotor.setPower(0);
                    robot.setFlywheelPower(0);
                })
        );
        sequence.init();
    }

    @Override
    public void loop() {
        sequence.loop();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }
}
