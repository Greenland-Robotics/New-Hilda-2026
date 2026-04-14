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
// ALL SHOTS occur from SHOOT_POSE (90, 96, 46°)
//
// SEQUENCE:
//   1.  Preload path → shoot pose → shoot
//   2.  Idle + intake → collect first spike mark → return to shoot pose → shoot
//   3.  Idle + intake → 2nd collect → return to shoot pose → shoot
//   4.  Park
//
// CHASSIS SPEED:
//   CollectFirstSpikeMark:
//     SpikeApproach     SHOOT_POSE → SPIKE_APPROACH_POSE    NORMAL
//     SpikeSweep        SPIKE_APPROACH_POSE → SPIKE_SWEEP_POSE   SLOW
//     SpikeCollectEnd   SPIKE_SWEEP_POSE → SPIKE_COLLECT_END_POSE SLOW
//     SpikeReturnShoot  SPIKE_COLLECT_END_POSE → SHOOT_POSE  NORMAL
//   2nd Collect:
//     Collect2Out       SHOOT_POSE → COLLECT2_OUT_POSE       NORMAL
//     Collect2Sweep     COLLECT2_OUT_POSE → COLLECT2_OUT_POSE SLOW
//     Collect2End       COLLECT2_OUT_POSE → COLLECT2_END_POSE NORMAL
//     Collect2ReturnShoot COLLECT2_END_POSE → SHOOT_POSE     NORMAL
// ═══════════════════════════════════════════════════════════════════════════

@Autonomous(name = "RedNearAutoGate", group = "Hilda Auto")
public class RedNearAutoGate extends AutoBase {

    // ─────────────────────────────────────────────────────────────────────
    // TIMING CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final int SETTLE_MS = 300;

    // ─────────────────────────────────────────────────────────────────────
    // DRIVE SPEED CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final double NORMAL_SPEED = 1.0;
    private static final double INTAKE_SPEED = 0.65;

    // ─────────────────────────────────────────────────────────────────────
    // START POSE
    // ─────────────────────────────────────────────────────────────────────
    private static final Pose START_POSE = new Pose(108, 132, Math.toRadians(0));

    // ─────────────────────────────────────────────────────────────────────
    // SHOOT POSE — shared target for all return-to-shoot tracker records
    // Change this one constant to update both paths and telemetry targets
    // ─────────────────────────────────────────────────────────────────────
    static final Pose SHOOT_POSE = new Pose(90.000, 96.000, Math.toRadians(46));

    Paths paths;
    PoseErrorTracker tracker = new PoseErrorTracker();

    // ─────────────────────────────────────────────────────────────────────
    // INITIALIZE
    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void initialize() {
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);
        gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        hoodServo.setPosition(Constants.Hood.MEDIUM);
    }

    // ─────────────────────────────────────────────────────────────────────
    // BUILD COMMANDS
    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void buildCommands() {
        commandRunner = new CommandRunner(
                new SeriesCommand(

                        // ══════════════════════════════════════════════
                        // PHASE 1: PRELOAD
                        // Drive to shoot pose with flywheel spinning up
                        // ══════════════════════════════════════════════
                        new ParallelCommand(
                                new InstantCommand(() -> {
                                    setFlywheelVelocity(Constants.Flywheel.VELOCITY_MEDIUM);
                                    hoodServo.setPosition(Constants.Hood.MEDIUM);
                                    follower.setMaxPower(NORMAL_SPEED);
                                }),
                                new FollowPath(paths.Preloadshoot)
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
                        new FollowPath(paths.SpikeReturnShoot),
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
                        new FollowPath(paths.Collect2ReturnShoot),
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

    // ─────────────────────────────────────────────────────────────────────
    // RUN LOOP
    // ─────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────
    // PATHS
    // Pose constants are static final — change a constant here and both
    // the path builder AND the tracker target update automatically.
    // ─────────────────────────────────────────────────────────────────────
    public static class Paths {

        // ── Endpoint pose constants ──
        static final Pose SPIKE_APPROACH_POSE    = new Pose(100.983, 82.064, Math.toRadians(0));
        static final Pose SPIKE_SWEEP_POSE       = new Pose(125.000, 81.867, Math.toRadians(0));
        static final Pose SPIKE_COLLECT_END_POSE = new Pose(126.571, 73.609, Math.toRadians(0));
        static final Pose COLLECT2_OUT_POSE      = new Pose(102.740, 58.815, Math.toRadians(0));
        static final Pose COLLECT2_END_POSE      = new Pose(130.817, 57.697, Math.toRadians(0));
        static final Pose PARK_POSE              = new Pose(126.605, 58.969, Math.toRadians(45));

        // ── PathChain declarations ──
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

            // ── Preload ──
            Preloadshoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            START_POSE,
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Collect First Spike Mark ──
            SpikeApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            new Pose(93.872, 79.306),
                            SPIKE_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), SPIKE_APPROACH_POSE.getHeading())
                    .build();

            SpikeSweep = follower.pathBuilder()
                    .addPath(new BezierLine(
                            SPIKE_APPROACH_POSE,
                            SPIKE_SWEEP_POSE))
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
                    .addPath(new BezierLine(
                            SPIKE_COLLECT_END_POSE,
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(SPIKE_COLLECT_END_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── 2nd Collect ──
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
                    .addPath(new BezierLine(
                            COLLECT2_OUT_POSE,
                            COLLECT2_END_POSE))
                    .setTangentHeadingInterpolation()
                    .build();

            Collect2ReturnShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            COLLECT2_END_POSE,
                            new Pose(110.729, 43.027),
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(COLLECT2_END_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Park ──
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