package gcsrobotics.opmode;

import static gcsrobotics.pedroPathing.Constants.SnapPositions.BLUE_FAR_START;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.GoToHumanPlayer;
import gcsrobotics.commands.KickstandSubsystem;
import gcsrobotics.commands.Park;
import gcsrobotics.commands.SetHoodAngle;
import gcsrobotics.commands.ShootCurrent;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.control.TeleOpBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.ButtonAction;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;

@TeleOp(name = "BLUE TeleOp — NH Premier", group = "Hilda")
public class BlueTeleopGearmo extends TeleOpBase {

    // ============================================================
    //  GAMEPAD 1 — DRIVER
    //  Left stick Y        Forward / back
    //  Left stick X        Strafe left / right
    //  Right stick X       Rotation
    //  Left bumper         Strafe LEFT (overrides stick)
    //  Right bumper        Strafe RIGHT (overrides stick)
    //  Left trigger        Strafe left (analog)
    //  Right trigger       Strafe right (analog)
    //  A                   Snap to CLOSE shooting position
    //  X                   Snap to MEDIUM shooting position
    //  Y                   Snap to TOP shooting position
    //  B                   Snap to FAR shooting position
    //  D-pad Up            Deploy kickstand (hold) / stop on release
    //  D-pad Down          Retract kickstand (hold) / stop on release
    //  D-pad Right         GoToHumanPlayer
    //  D-pad Left          Park
    //  Back (Share)        Reset Pedro pose to BLUE_RESET_POSE (human player corner)
    //
    //  GAMEPAD 2 — OPERATOR
    //  Left stick Y        Intake (forward = in, pull back = unjam)
    //                      Suppressed during shoot sequence
    //  Left trigger        Rotate chassis LEFT at 0.5 speed
    //  Right trigger       Rotate chassis RIGHT at 0.5 speed
    //  A                   Hood → CLOSE  + flywheel CLOSE (slowest)
    //  X                   Hood → MEDIUM + flywheel MEDIUM
    //  Y                   Hood → TOP    + flywheel TOP
    //  B                   Hood → FAR    + flywheel FAR (fastest)
    //  Right bumper        Shoot at current position
    //  Left bumper         Flywheel → IDLE speed
    //  Back button         Flywheel → ZERO (full stop)
    //  Right stick Y       Manual flywheel speed (0–6000 RPM)
    //  D-pad up            Add 10 RPM to all shot velocities (cumulative)
    //  D-pad down          Subtract 10 RPM from all shot velocities (cumulative)
    // ============================================================

    // ---- Alliance — hardcoded BLUE ----
    private final boolean isBlue = true;

    // ---- Current shooting position (updated by GP2 A/X/Y/B) ----
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // ---- Global RPM offset — adjusted via GP2 D-pad up/down, resets on restart ----
    private double rpmOffset = 0.0;

    // ---- D-pad edge detection for RPM offset ----
    private boolean prevDpadUp   = false;
    private boolean prevDpadDown = false;

    // ---- Kickstand ----
    private KickstandSubsystem kickstand;

    // ---- GP1 button actions ----
    private ButtonAction snapClose;
    private ButtonAction snapMedium;
    private ButtonAction snapTop;
    private ButtonAction snapFar;
    private ButtonAction goToHumanPlayer;
    private ButtonAction pathToPark;
    private ButtonAction resetPose;

    // ---- GP2 button actions ----
    private ButtonAction hoodClose;
    private ButtonAction hoodMedium;
    private ButtonAction hoodTop;
    private ButtonAction hoodFar;
    private ButtonAction shoot;
    private ButtonAction idleFlywheel;
    private ButtonAction zeroFlywheel;

    // ---- Drive motors ----
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double speed = 1.0;

    @Override
    protected void initialize() {
        driveMode = false;

        kickstand = new KickstandSubsystem(
                hardwareMap.get(CRServo.class, Constants.Kickstand.MOTOR_NAME)
        );

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        follower.setStartingPose(BLUE_FAR_START);

        buildActions();

        telemetry.addData("Status",   "Initialized");
        telemetry.addData("Alliance", "BLUE");
        telemetry.update();
    }

    private void buildActions() {

        // ---- GP1: Snap to BLUE shooting positions ----
        snapClose = new ButtonAction(
                () -> new ParallelCommand(
                        new FollowPath(
                                follower.getPose(),
                                Constants.SnapPositions.BLUE_CLOSE),
                        new SeriesCommand(
                                new InstantCommand(() -> {
                                    currentPosition = ShootingPosition.CLOSE;
                                    setFlywheelVelocity(
                                            Constants.Flywheel.VELOCITY_CLOSE + rpmOffset);
                                }),
                                new SetHoodAngle(ShootingPosition.CLOSE)
                        )
                ), commandRunner
        );
        snapMedium = new ButtonAction(
                () -> new ParallelCommand(
                        new FollowPath(
                                follower.getPose(),
                                Constants.SnapPositions.BLUE_MEDIUM),
                        new SeriesCommand(
                                new InstantCommand(() -> {
                                    currentPosition = ShootingPosition.MEDIUM;
                                    setFlywheelVelocity(
                                            Constants.Flywheel.VELOCITY_MEDIUM + rpmOffset);
                                }),
                                new SetHoodAngle(ShootingPosition.MEDIUM)
                        )
                ), commandRunner
        );
        snapTop = new ButtonAction(
                () -> new ParallelCommand(
                        new FollowPath(
                                follower.getPose(),
                                Constants.SnapPositions.BLUE_TOP),
                        new SeriesCommand(
                                new InstantCommand(() -> {
                                    currentPosition = ShootingPosition.TOP;
                                    setFlywheelVelocity(
                                            Constants.Flywheel.VELOCITY_TOP + rpmOffset);
                                }),
                                new SetHoodAngle(ShootingPosition.TOP)
                        )
                ), commandRunner
        );
        snapFar = new ButtonAction(
                () -> new ParallelCommand(
                        new FollowPath(
                                follower.getPose(),
                                Constants.SnapPositions.BLUE_FAR),
                        new SeriesCommand(
                                new InstantCommand(() -> {
                                    currentPosition = ShootingPosition.FAR;
                                    setFlywheelVelocity(
                                            Constants.Flywheel.VELOCITY_FAR + rpmOffset);
                                }),
                                new SetHoodAngle(ShootingPosition.FAR)
                        )
                ), commandRunner
        );

        // ---- GP1: D-pad ----
        goToHumanPlayer = new ButtonAction(
                () -> new GoToHumanPlayer(isBlue), commandRunner
        );
        pathToPark = new ButtonAction(
                () -> new Park(isBlue), commandRunner
        );

        // ---- GP1: Back (Share) — reset Pinpoint pose to known human player corner ----
        // Robot must be physically positioned at the reset pose before pressing
        resetPose = new ButtonAction(
                () -> new InstantCommand(() -> {
                    Pose rp = Constants.SnapPositions.BLUE_RESET_POSE;
                    odo.setPosition(new Pose2D(
                            DistanceUnit.INCH,
                            rp.getX(),
                            rp.getY(),
                            AngleUnit.RADIANS,
                            rp.getHeading()
                    ));
                }),
                commandRunner
        );

        // ---- GP2: Hood position + flywheel speed ----
        hoodClose = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.CLOSE;
                            setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_CLOSE + rpmOffset);
                        }),
                        new SetHoodAngle(ShootingPosition.CLOSE)
                ), commandRunner
        );
        hoodMedium = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.MEDIUM;
                            setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_MEDIUM + rpmOffset);
                        }),
                        new SetHoodAngle(ShootingPosition.MEDIUM)
                ), commandRunner
        );
        hoodTop = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.TOP;
                            setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_TOP + rpmOffset);
                        }),
                        new SetHoodAngle(ShootingPosition.TOP)
                ), commandRunner
        );
        hoodFar = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.FAR;
                            setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_FAR + rpmOffset);
                        }),
                        new SetHoodAngle(ShootingPosition.FAR)
                ), commandRunner
        );

        shoot = new ButtonAction(
                () -> new ShootCurrent(() -> currentPosition), commandRunner
        );

        idleFlywheel = new ButtonAction(
                () -> new InstantCommand(() ->
                        setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE)),
                commandRunner
        );

        zeroFlywheel = new ButtonAction(
                () -> new InstantCommand(() -> setFlywheelVelocity(0)),
                commandRunner
        );
    }

    @Override
    protected void runLoop() {
        updateFlywheel();

        boolean objectDetected = isBallAtTransfer();
        if (objectDetected) {
            ledLight.setPosition(Constants.LED.SOLID_GREEN);
        } else {
            ledLight.setPosition(Constants.LED.OFF);
        }

        double horizontal;
        if (gamepad1.left_bumper) {
            horizontal = -1.0;
        } else if (gamepad1.right_bumper) {
            horizontal = 1.0;
        } else {
            double stick = Math.abs(gamepad1.left_stick_x) > 0.1
                    ? gamepad1.left_stick_x : 0;
            double rTrig = Math.abs(gamepad1.right_trigger) > 0.1
                    ? gamepad1.right_trigger : 0;
            double lTrig = Math.abs(gamepad1.left_trigger) > 0.1
                    ? gamepad1.left_trigger : 0;
            horizontal = stick + rTrig - lTrig;
        }

        double forward = Math.abs(gamepad1.left_stick_y) > 0.1
                ? -gamepad1.left_stick_y : 0;

        double pivot = Math.abs(gamepad1.right_stick_x) > 0.1
                ? gamepad1.right_stick_x : 0;

        if (gamepad2.right_trigger > 0.1) {
            pivot += gamepad2.right_trigger * 0.5;
        } else if (gamepad2.left_trigger > 0.1) {
            pivot -= gamepad2.left_trigger * 0.5;
        }

        pivot = Math.max(-1.0, Math.min(1.0, pivot));

        if (!follower.isBusy()) {
            fl.setPower(speed * (forward + horizontal + pivot));
            fr.setPower(speed * (forward - horizontal - pivot));
            bl.setPower(speed * (forward - horizontal + pivot));
            br.setPower(speed * (forward + horizontal - pivot));
        }

        snapClose.update(gamepad1.a && !gamepad1.start);
        snapMedium.update(gamepad1.x && !gamepad1.start);
        snapTop.update(gamepad1.y && !gamepad1.start);
        snapFar.update(gamepad1.b && !gamepad1.start);

        goToHumanPlayer.update(gamepad1.dpad_right);
        pathToPark.update(gamepad1.dpad_left);

        // Pose reset — robot must be physically at human player corner
        resetPose.update(gamepad1.back);

        if (gamepad1.dpad_up) {
            kickstand.deploy();
        } else if (gamepad1.dpad_down) {
            kickstand.retract();
        } else {
            kickstand.stop();
        }

        if (commandRunner.isFinished()) {
            double intakePower = gamepad2.left_stick_y;
            if (Math.abs(intakePower) > 0.1) {
                intakeMotor.setPower(intakePower);
            } else {
                intakeMotor.setPower(0);
            }
        }

        hoodClose.update(gamepad2.a);
        hoodMedium.update(gamepad2.x);
        hoodTop.update(gamepad2.y);
        hoodFar.update(gamepad2.b);

        shoot.update(gamepad2.right_bumper);
        idleFlywheel.update(gamepad2.left_bumper);
        zeroFlywheel.update(gamepad2.back);

        double manualFlywheel = -gamepad2.right_stick_y;
        if (Math.abs(manualFlywheel) > 0.1) {
            setFlywheelVelocity(manualFlywheel * 6000.0);
        }

        boolean dpadUp   = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        if (dpadUp && !prevDpadUp) {
            rpmOffset += 10.0;
            setFlywheelVelocity(currentPosition.targetVelocity + rpmOffset);
        }
        if (dpadDown && !prevDpadDown) {
            rpmOffset -= 10.0;
            setFlywheelVelocity(currentPosition.targetVelocity + rpmOffset);
        }
        prevDpadUp   = dpadUp;
        prevDpadDown = dpadDown;

        telemetry.addData("Artifact Detected", objectDetected ? "YES" : "NO");
        telemetry.addData("Shot Pose",         currentPosition);
        telemetry.addData("Target RPM",        currentPosition.targetVelocity + rpmOffset);
        telemetry.addData("RPM Offset",        rpmOffset);
        telemetry.addData("Actual RPM",        getFlywheelVelocity());
        telemetry.addLine("---");
        telemetry.addData("Pedro X",           "%.2f", follower.getPose().getX());
        telemetry.addData("Pedro Y",           "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)",     "%.1f",
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}