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
import gcsrobotics.control.PoseErrorTracker;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

// ═══════════════════════════════════════════════════════════════════════════
// RedFarAuto — Far Zone Autonomous (Red Alliance)
//
// CHASSIS SPEED CHANGES:
//   SpikeMarkCycle:
//     - Approach curve       → NORMAL_SPEED
//     - Collect sweep        → INTAKE_SPEED  (slow)
//     - Return               → NORMAL_SPEED
//   CornerCycle (x2):
//     - Approach (tangent)   → NORMAL_SPEED
//     - Bump forward/back    → INTAKE_SPEED  (slow)
//     - Return               → NORMAL_SPEED
// ═══════════════════════════════════════════════════════════════════════════

@Autonomous(name = "RedFarAuto")
public class RedFarAuto extends AutoBase {

    // ─────────────────────────────────────────────────────────────────────
    // TIMING CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final int SPINUP_MS     = 2000;
    private static final int STABILIZE_MS  = 300;
    private static final int BUMP_PAUSE_MS = 500;

    // ─────────────────────────────────────────────────────────────────────
    // DRIVE SPEED CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final double NORMAL_SPEED = 0.95;
    private static final double INTAKE_SPEED = 0.65;

    // ─────────────────────────────────────────────────────────────────────
    // START POSE
    // ─────────────────────────────────────────────────────────────────────
    private static final Pose START_POSE = new Pose(80, 9, Math.toRadians(90));

    // ─────────────────────────────────────────────────────────────────────
    // SHOOT POSE
    // ─────────────────────────────────────────────────────────────────────
    static final Pose SHOOT_POSE = new Pose(85.000, 12.000, Math.toRadians(64));

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
        hoodServo.setPosition(Constants.Hood.FAR);
    }

    // ─────────────────────────────────────────────────────────────────────
    // BUILD COMMANDS
    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void buildCommands() {
        commandRunner = new CommandRunner(
                new SeriesCommand(

                        // ── STEP 1: Spin up flywheel + set hood
                        new InstantCommand(() -> {
                            setFlywheelVelocity(Constants.Flywheel.VELOCITY_FAR);
                            hoodServo.setPosition(Constants.Hood.FAR);
                            follower.setMaxPower(NORMAL_SPEED);
                        }),

                        // ── STEP 2: Drive to shoot pose
                        new FollowPath(paths.PreloadShoot),

                        // ── STEP 3: Wait for flywheel to reach speed
                        new SleepCommand(SPINUP_MS),

                        // ── STEP 4: Shoot preloads
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("PRELOAD_SHOOT", follower.getPose(), SHOOT_POSE)),

                        // ── STEP 5: Idle flywheel + start intake
                        new InstantCommand(() ->
                                setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE)),
                        new StartIntake(),

                        // ══════════════════════════════════════════════
                        // CORNER CYCLE 1
                        // ══════════════════════════════════════════════

                        new FollowPath(paths.CornerApproach),
                        new InstantCommand(() -> tracker.record("CORNER_APPROACH", follower.getPose(), Paths.CORNER_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.CornerBumpBack),
                        new InstantCommand(() -> tracker.record("CORNER_BUMP_BACK", follower.getPose(), Paths.CORNER_BUMP_BACK_POSE)),
                        new FollowPath(paths.CornerBumpForward),
                        new InstantCommand(() -> tracker.record("CORNER_BUMP_FWD", follower.getPose(), Paths.CORNER_BUMP_FWD_POSE)),
                        new FollowPath(paths.CornerAdjust),
                        new InstantCommand(() -> tracker.record("CORNER_ADJUST", follower.getPose(), Paths.CORNER_ADJUST_POSE)),

                        new SleepCommand(BUMP_PAUSE_MS),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),

                        new ParallelCommand(
                                new FollowPath(paths.CornerReturn),
                                new InstantCommand(() ->
                                        setFlywheelVelocity(Constants.Flywheel.VELOCITY_FAR))
                        ),
                        new SleepCommand(STABILIZE_MS),
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("CORNER1_SHOOT", follower.getPose(), SHOOT_POSE)),
                        new StartIntake(),
                        new InstantCommand(() ->
                                setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE)),

                        // ══════════════════════════════════════════════
                        // SPIKE MARK CYCLE
                        // ══════════════════════════════════════════════

                        new FollowPath(paths.SpikeApproach),
                        new InstantCommand(() -> tracker.record("SPIKE_APPROACH", follower.getPose(), Paths.SPIKE_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.SpikeSweep),
                        new InstantCommand(() -> tracker.record("SPIKE_SWEEP_END", follower.getPose(), Paths.SPIKE_SWEEP_END_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),

                        new ParallelCommand(
                                new FollowPath(paths.SpikeReturn),
                                new InstantCommand(() ->
                                        setFlywheelVelocity(Constants.Flywheel.VELOCITY_FAR))
                        ),
                        new SleepCommand(STABILIZE_MS),
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("SPIKE_SHOOT", follower.getPose(), SHOOT_POSE)),
                        new StartIntake(),
                        new InstantCommand(() ->
                                setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE)),

                        // ══════════════════════════════════════════════
                        // CORNER CYCLE 2
                        // ══════════════════════════════════════════════

                        new FollowPath(paths.CornerApproach),
                        new InstantCommand(() -> tracker.record("CORNER2_APPROACH", follower.getPose(), Paths.CORNER_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.CornerBumpBack),
                        new InstantCommand(() -> tracker.record("CORNER2_BUMP_BACK", follower.getPose(), Paths.CORNER_BUMP_BACK_POSE)),
                        new FollowPath(paths.CornerBumpForward),
                        new InstantCommand(() -> tracker.record("CORNER2_BUMP_FWD", follower.getPose(), Paths.CORNER_BUMP_FWD_POSE)),
                        new FollowPath(paths.CornerAdjust),
                        new InstantCommand(() -> tracker.record("CORNER_ADJUST", follower.getPose(), Paths.CORNER_ADJUST_POSE)),

                        new SleepCommand(BUMP_PAUSE_MS),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),

                        new ParallelCommand(
                                new FollowPath(paths.CornerReturn),
                                new InstantCommand(() ->
                                        setFlywheelVelocity(Constants.Flywheel.VELOCITY_FAR))
                        ),
                        new SleepCommand(STABILIZE_MS),
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("CORNER2_SHOOT", follower.getPose(), SHOOT_POSE)),

                        // ── FINAL: Stop intake + park
                        new InstantCommand(() -> {
                            intakeMotor.setPower(0);
                            setFlywheelVelocity(0);
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
    static class Paths {

        // ── Endpoint pose constants ──
        static final Pose CORNER_APPROACH_POSE  = new Pose(124.000,  8.700, Math.toRadians(0));
        static final Pose CORNER_BUMP_BACK_POSE = new Pose(120.383,  8.564, Math.toRadians(0));
        static final Pose CORNER_BUMP_FWD_POSE  = new Pose(124.000,  8.600, Math.toRadians(0));
        static final Pose SPIKE_APPROACH_POSE   = new Pose( 99.952, 34.736, Math.toRadians(0));
        static final Pose SPIKE_SWEEP_END_POSE  = new Pose(124.433, 35.190, Math.toRadians(0));
        static final Pose PARK_POSE             = new Pose(112.749,  8.708, Math.toRadians(0));

        static final Pose CORNER_ADJUST_POSE = new Pose(124, 12.6, Math.toRadians(0));

        // ── PathChain declarations ──
        public PathChain PreloadShoot;
        public PathChain SpikeApproach;
        public PathChain SpikeSweep;
        public PathChain SpikeReturn;
        public PathChain CornerApproach;
        public PathChain CornerBumpForward;
        public PathChain CornerBumpBack;
        public PathChain CornerReturn;
        public PathChain Park;

        public PathChain CornerAdjust;

        public Paths(Follower follower) {

            // ── Preload ──
            PreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            START_POSE,
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Corner Cycle (shared by both cycles) ──
            CornerApproach = follower.pathBuilder()
                    .addPath(new BezierLine(
                            SHOOT_POSE,
                            CORNER_APPROACH_POSE))
                    .setTangentHeadingInterpolation()
                    .build();

            CornerBumpBack = follower.pathBuilder()
                    .addPath(new BezierLine(
                            CORNER_APPROACH_POSE,
                            CORNER_BUMP_BACK_POSE))
                    .setLinearHeadingInterpolation(CORNER_APPROACH_POSE.getHeading(), CORNER_BUMP_BACK_POSE.getHeading())
                    .build();

            CornerBumpForward = follower.pathBuilder()
                    .addPath(new BezierLine(
                            CORNER_BUMP_BACK_POSE,
                            CORNER_BUMP_FWD_POSE))
                    .setLinearHeadingInterpolation(CORNER_BUMP_BACK_POSE.getHeading(), CORNER_BUMP_FWD_POSE.getHeading())
                    .build();

            CornerAdjust = follower.pathBuilder()
                    .addPath(new BezierLine(
                            CORNER_BUMP_FWD_POSE,
                            CORNER_ADJUST_POSE))
                    .setLinearHeadingInterpolation(CORNER_BUMP_FWD_POSE.getHeading(), CORNER_ADJUST_POSE.getHeading())
                    .build();

            CornerReturn = follower.pathBuilder()
                    .addPath(new BezierLine(
                            CORNER_ADJUST_POSE,
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(CORNER_ADJUST_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Spike Mark Cycle ──
            SpikeApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE,
                            new Pose(90.434, 36.425),
                            SPIKE_APPROACH_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(70), SPIKE_APPROACH_POSE.getHeading())
                    .build();

            SpikeSweep = follower.pathBuilder()
                    .addPath(new BezierLine(
                            SPIKE_APPROACH_POSE,
                            SPIKE_SWEEP_END_POSE))
                    .setLinearHeadingInterpolation(SPIKE_APPROACH_POSE.getHeading(), SPIKE_SWEEP_END_POSE.getHeading())
                    .build();

            SpikeReturn = follower.pathBuilder()
                    .addPath(new BezierLine(
                            SPIKE_SWEEP_END_POSE,
                            SHOOT_POSE))
                    .setLinearHeadingInterpolation(SPIKE_SWEEP_END_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Park ──
            Park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            SHOOT_POSE,
                            PARK_POSE))
                    .setLinearHeadingInterpolation(Math.toRadians(70), PARK_POSE.getHeading())
                    .build();
        }
    }
}