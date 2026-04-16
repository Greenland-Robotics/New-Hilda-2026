package gcsrobotics.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.ShootCurrent;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.commands.StartIntake;
import gcsrobotics.control.AutoBase;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.control.PoseErrorTracker;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

// ═══════════════════════════════════════════════════════════════════════════
// RedNearAutoGate — Near Zone Autonomous (Red Alliance)
//
// holdEnd = true on all return-to-shoot paths for active endpoint correction
// ═══════════════════════════════════════════════════════════════════════════

@Autonomous(name = "RedNearAutoGate", group = "Hilda Auto")
public class RedNearAutoGate extends AutoBase {

    private static final int SETTLE_MS = 300;

    private static final double NORMAL_SPEED = 1.0;
    private static final double INTAKE_SPEED = 0.65;

    private static final Pose START_POSE = new Pose(123, 116, Math.toRadians(37));

    // SHOOT_POSE heading nudged to 45° to reduce heading overshoot
    static final Pose SHOOT_POSE = new Pose(90.000, 96.000, Math.toRadians(40));

    Paths paths;
    PoseErrorTracker tracker = new PoseErrorTracker();

    @Override
    protected void initialize() {
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);
        gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        hoodServo.setPosition(Constants.Hood.MEDIUM);
    }

    @Override
    protected void buildCommands() {
        commandRunner = new CommandRunner(
                new SeriesCommand(

                        // ══════════════════════════════════════════════
                        // PHASE 1: PRELOAD
                        // ══════════════════════════════════════════════
                        new ParallelCommand(
                                new InstantCommand(() -> {
                                    setFlywheelVelocity(Constants.Flywheel.VELOCITY_MEDIUM);
                                    hoodServo.setPosition(Constants.Hood.MEDIUM);
                                    follower.setMaxPower(NORMAL_SPEED);
                                }),
                                // holdEnd = true — hold at shoot pose during settle
                                new FollowPath(paths.Preloadshoot, true)
                        ),
                        new SleepCommand(SETTLE_MS),
                        new ShootCurrent(() -> ShootingPosition.MEDIUM),
                        new InstantCommand(() -> tracker.record("PRELOAD_SHOOT", follower.getPose(), SHOOT_POSE)),

                        // ══════════════════════════════════════════════
                        // PHASE 2: COLLECT FIRST SPIKE MARK
                        // ══════════════════════════════════════════════

                        new InstantCommand(() ->
                                setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE)),
                        new StartIntake(),

                        new FollowPath(paths.SpikeApproach),
                        new InstantCommand(() -> tracker.record("SPIKE_APPROACH", follower.getPose(), Paths.SPIKE_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.SpikeSweep),
                        new InstantCommand(() -> tracker.record("SPIKE_SWEEP", follower.getPose(), Paths.SPIKE_SWEEP_POSE)),
                        new FollowPath(paths.SpikeCollectEnd),
                        new InstantCommand(() -> tracker.record("SPIKE_COLLECT_END", follower.getPose(), Paths.SPIKE_COLLECT_END_POSE)),

                        new InstantCommand(() -> {
                            follower.setMaxPower(NORMAL_SPEED);
                            setFlywheelVelocity(Constants.Flywheel.VELOCITY_MEDIUM);
                            hoodServo.setPosition(Constants.Hood.MEDIUM);
                        }),
                        // holdEnd = true — hold at shoot pose during settle + shoot
                        new FollowPath(paths.SpikeReturnShoot, true),
                        new InstantCommand(() -> tracker.record("SPIKE_RETURN_SHOOT", follower.getPose(), SHOOT_POSE)),

                        new InstantCommand(() -> intakeMotor.setPower(0)),
                        new SleepCommand(SETTLE_MS),
                        new ShootCurrent(() -> ShootingPosition.MEDIUM),

                        // ══════════════════════════════════════════════
                        // PHASE 3: 2ND COLLECT
                        // ══════════════════════════════════════════════

                        new InstantCommand(() ->
                                setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE)),
                        new StartIntake(),

                        new FollowPath(paths.Collect2Out),
                        new InstantCommand(() -> tracker.record("COLLECT2_OUT", follower.getPose(), Paths.COLLECT2_OUT_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.Collect2Sweep),
                        new InstantCommand(() -> tracker.record("COLLECT2_SWEEP", follower.getPose(), Paths.COLLECT2_OUT_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),
                        new FollowPath(paths.Collect2End),
                        new InstantCommand(() -> tracker.record("COLLECT2_END", follower.getPose(), Paths.COLLECT2_END_POSE)),

                        new InstantCommand(() -> {
                            setFlywheelVelocity(Constants.Flywheel.VELOCITY_MEDIUM);
                            hoodServo.setPosition(Constants.Hood.MEDIUM);
                        }),
                        // holdEnd = true — hold at shoot pose during settle + shoot
                        new FollowPath(paths.Collect2ReturnShoot, true),
                        new InstantCommand(() -> tracker.record("COLLECT2_RETURN_SHOOT", follower.getPose(), SHOOT_POSE)),

                        new InstantCommand(() -> intakeMotor.setPower(0)),
                        new SleepCommand(SETTLE_MS),
                        new ShootCurrent(() -> ShootingPosition.MEDIUM),

                        // ══════════════════════════════════════════════
                        // PHASE 4: PARK
                        // ══════════════════════════════════════════════
                        new InstantCommand(() -> {
                            setFlywheelVelocity(0);
                            intakeMotor.setPower(0);
                            gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
                            follower.setMaxPower(NORMAL_SPEED);
                        }),
                        new FollowPath(paths.Park),
                        new InstantCommand(() -> tracker.record("PARK_END", follower.getPose(), Paths.PARK_POSE))
                )
        );
    }

    @Override
    protected void runLoop() {
        updateFlywheel();
        telemetry.addData("Pedro X",      "%.2f", follower.getPose().getX());
        telemetry.addData("Pedro Y",      "%.2f", follower.getPose().getY());
        telemetry.addData("Heading",      "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Flywheel RPM", "%.0f", getFlywheelVelocity());
        telemetry.addData("Auto Done",    commandRunner.isFinished());
        telemetry.addLine("---");
        tracker.display(telemetry);
        telemetry.update();
    }

    public static class Paths {

        static final Pose SPIKE_APPROACH_POSE    = new Pose(100.983, 80.5, Math.toRadians(0));
        static final Pose SPIKE_SWEEP_POSE       = new Pose(125.000, 80.5, Math.toRadians(0));
        static final Pose SPIKE_COLLECT_END_POSE = new Pose(126.571, 71, Math.toRadians(0));
        static final Pose COLLECT2_OUT_POSE      = new Pose(102.740, 57, Math.toRadians(0));
        static final Pose COLLECT2_END_POSE      = new Pose(130.817, 57, Math.toRadians(0));
        static final Pose PARK_POSE              = new Pose(126.605, 50, Math.toRadians(45));

        public PathChain Preloadshoot;
        public PathChain SpikeApproach;
        public PathChain SpikeSweep;
        public PathChain SpikeCollectEnd;
        public PathChain SpikeReturnShoot;
        public PathChain Collect2Out;
        public PathChain Collect2Sweep;
        public PathChain Collect2End;
        public PathChain Collect2ReturnShoot;
        public PathChain Park;

        public Paths(Follower follower) {

            Preloadshoot = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            SpikeApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            new Pose(93.872, 79.306),
                            SPIKE_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), SPIKE_APPROACH_POSE.getHeading())
                    .build();

            SpikeSweep = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE_APPROACH_POSE, SPIKE_SWEEP_POSE))
                    .setLinearHeadingInterpolation(SPIKE_APPROACH_POSE.getHeading(), SPIKE_SWEEP_POSE.getHeading())
                    .build();

            SpikeCollectEnd = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SPIKE_SWEEP_POSE,
                            new Pose(111.524, 71.990),
                            SPIKE_COLLECT_END_POSE))
                    .setLinearHeadingInterpolation(SPIKE_SWEEP_POSE.getHeading(), SPIKE_COLLECT_END_POSE.getHeading())
                    .build();

            SpikeReturnShoot = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE_COLLECT_END_POSE, SHOOT_POSE))
                    .setLinearHeadingInterpolation(SPIKE_COLLECT_END_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            Collect2Out = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            new Pose(94.092, 56.996),
                            COLLECT2_OUT_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT2_OUT_POSE.getHeading())
                    .build();

            Collect2Sweep = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            new Pose(94.092, 56.996),
                            COLLECT2_OUT_POSE))
                    .setLinearHeadingInterpolation(COLLECT2_OUT_POSE.getHeading(), COLLECT2_OUT_POSE.getHeading())
                    .build();

            Collect2End = follower.pathBuilder()
                    .addPath(new BezierLine(COLLECT2_OUT_POSE, COLLECT2_END_POSE))
                    .setTangentHeadingInterpolation()
                    .build();

            Collect2ReturnShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            COLLECT2_END_POSE,
                            new Pose(110.729, 43.027),
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(COLLECT2_END_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            new Pose(121.990, 50.242),
                            PARK_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}