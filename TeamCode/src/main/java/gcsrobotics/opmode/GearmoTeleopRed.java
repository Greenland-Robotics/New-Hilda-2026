package gcsrobotics.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import gcsrobotics.commands.DeployKickstandCommand;
import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.KickstandSubsystem;
import gcsrobotics.commands.RetractKickstandCommand;
import gcsrobotics.commands.SetHoodAngle;
import gcsrobotics.commands.ShootCurrent;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.control.TeleOpBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.ButtonAction;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

// ============================================================
//  Gearmo TeleOp — Red
// ============================================================
//
//  GAMEPAD 1 — DRIVER
//  ----------------------------------------------------------
//  Left stick          Manual drive (robot-centric)
//  Right stick X       Manual rotation
//  A (hold)            Pedro → CLOSE position, then auto spinup
//  B (hold)            Pedro → MEDIUM position, then auto spinup
//  X (hold)            Pedro → FAR position, then auto spinup
//  Y (hold)            Pedro → TOP position, then auto spinup
//  Any stick input     Cancels Pedro path, returns to manual
//  D-pad Up            Deploy kickstand
//  D-pad Down          Retract kickstand
//  Start + A           Set alliance → BLUE
//  Start + B           Set alliance → RED
//
//  GAMEPAD 2 — OPERATOR
//  ----------------------------------------------------------
//  Left stick Y        Intake (forward = in, back = reverse)
//  A                   Spin up flywheel + set hood → CLOSE
//  B                   Spin up flywheel + set hood → MEDIUM
//  X                   Spin up flywheel + set hood → FAR
//  Y                   Spin up flywheel + set hood → TOP
//  Left bumper         Shoot at current position
//  Right bumper        Shoot at current position
//  Left trigger        Open gate manually
//  Right trigger       Close gate manually
//
// ============================================================

@TeleOp(name = "Gearmo TeleOp — Red", group = "Gearmo")
public class GearmoTeleopRed extends TeleOpBase {

    // ---- Alliance ----
    private boolean isBlue = false;

    // ---- Current shooting position ----
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // ---- Pedro path state ----
    private boolean pedroActive = false;
    private ShootingPosition pedroTarget = null;

    // ---- Kickstand ----
    private KickstandSubsystem kickstand;

    // ---- Command runner ----
    private CommandRunner commandRunner;

    // ---- GP2 button actions ----
    private ButtonAction spinUpClose;
    private ButtonAction spinUpMedium;
    private ButtonAction spinUpFar;
    private ButtonAction spinUpTop;
    private ButtonAction shootLeft;
    private ButtonAction shootRight;

    // ---- GP1 kickstand actions ----
    private ButtonAction deployKickstand;
    private ButtonAction retractKickstand;

    @Override
    protected void initialize() {
        kickstand = new KickstandSubsystem(hardwareMap.get(CRServo.class, "kickstand"));
        commandRunner = new CommandRunner();

        buildActions();

        // Initialize Pedro in teleop drive mode
        follower.startTeleopDrive();

        telemetry.addData("Gearmo TeleOp", "Initialized");
        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.update();
    }

    private void buildActions() {

        // ---- GP2: Spin up flywheel + set hood ----
        spinUpClose = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.CLOSE;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_CLOSE);
                        }),
                        new SetHoodAngle(ShootingPosition.CLOSE)
                ), commandRunner
        );
        spinUpMedium = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.MEDIUM;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_MEDIUM);
                        }),
                        new SetHoodAngle(ShootingPosition.MEDIUM)
                ), commandRunner
        );
        spinUpFar = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.FAR;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_FAR);
                        }),
                        new SetHoodAngle(ShootingPosition.FAR)
                ), commandRunner
        );
        spinUpTop = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.TOP;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_TOP);
                        }),
                        new SetHoodAngle(ShootingPosition.TOP)
                ), commandRunner
        );

        // ---- GP2: Shoot using live currentPosition supplier ----
        shootLeft  = new ButtonAction(
                new ShootCurrent(() -> currentPosition), commandRunner);
        shootRight = new ButtonAction(
                new ShootCurrent(() -> currentPosition), commandRunner);

        // ---- GP1: Kickstand ----
        deployKickstand  = new ButtonAction(
                new DeployKickstandCommand(kickstand), commandRunner);
        retractKickstand = new ButtonAction(
                new RetractKickstandCommand(kickstand), commandRunner);
    }

    @Override
    protected void runLoop() {
        OpModeBase robot = OpModeBase.INSTANCE;

        // ---- Alliance selection ----
        if (gamepad1.start && gamepad1.a) {
            if (!isBlue) { isBlue = true;  buildActions(); }
        } else if (gamepad1.start && gamepad1.b) {
            if (isBlue)  { isBlue = false; buildActions(); }
        }

        // ---- Joystick input detection ----
        boolean driverMoving =
                Math.abs(gamepad1.left_stick_x)  > 0.05 ||
                        Math.abs(gamepad1.left_stick_y)  > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;

        // ---- GP1: Pedro path buttons (hold to drive, release = stop) ----
        ShootingPosition heldPosition = null;
        if (!gamepad1.start) {
            if      (gamepad1.a) heldPosition = ShootingPosition.CLOSE;
            else if (gamepad1.b) heldPosition = ShootingPosition.MEDIUM;
            else if (gamepad1.x) heldPosition = ShootingPosition.FAR;
            else if (gamepad1.y) heldPosition = ShootingPosition.TOP;
        }

        if (driverMoving) {
            // Joystick input — cancel Pedro, return to manual
            if (pedroActive) {
                follower.breakFollowing();
                follower.startTeleopDrive();
                pedroActive = false;
                pedroTarget = null;
            }
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true
            );
            follower.update();

        } else if (heldPosition != null) {
            // Button held — start or continue Pedro path to this position
            if (!pedroActive || heldPosition != pedroTarget) {
                pedroTarget = heldPosition;
                pedroActive = true;
                Pose targetPose = new Pose(
                        pedroTarget.poseX,
                        pedroTarget.poseY,
                        pedroTarget.poseHeading
                );
                commandRunner.run(new FollowPath(targetPose));
            }
            follower.update();

        } else {
            // No button, no stick — idle
            if (pedroActive) {
                follower.breakFollowing();
                follower.startTeleopDrive();
                pedroActive = false;

                if (pedroTarget != null) {
                    currentPosition = pedroTarget;
                    robot.setFlywheelVelocity(pedroTarget.targetVelocity);
                    robot.hoodServo.setPosition(pedroTarget.hoodPosition);
                }
                pedroTarget = null;
            }
            follower.setTeleOpDrive(0, 0, 0, true);
            follower.update();
        }

        // ---- GP1: Kickstand ----
        deployKickstand.update(gamepad1.dpad_up);
        retractKickstand.update(gamepad1.dpad_down);

        // ---- GP2: Intake ----
        double intakeInput = -gamepad2.left_stick_y;
        robot.intakeMotor.setPower(Math.abs(intakeInput) > 0.05 ? intakeInput : 0);

        // ---- GP2: Spin up flywheel + hood ----
        spinUpClose.update(gamepad2.a);
        spinUpMedium.update(gamepad2.b);
        spinUpFar.update(gamepad2.x);
        spinUpTop.update(gamepad2.y);

        // ---- GP2: Shoot ----
        shootLeft.update(gamepad2.left_bumper);
        shootRight.update(gamepad2.right_bumper);

        // ---- GP2: Manual gate override ----
        if (gamepad2.left_trigger > 0.1) {
            robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
        } else if (gamepad2.right_trigger > 0.1) {
            robot.gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        }

        // ---- Command runner tick ----
        commandRunner.update();

        // ---- Telemetry ----
        telemetry.addData("Alliance",      isBlue ? "BLUE" : "RED");
        telemetry.addData("Position",      currentPosition);
        telemetry.addData("Pedro Active",  pedroActive);
        telemetry.addData("Flywheel Vel",  robot.getFlywheelVelocity());
        telemetry.addData("Intake Power",  robot.intakeMotor.getPower());
        telemetry.addData("X",             robot.follower.getPose().getX());
        telemetry.addData("Y",             robot.follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.update();
    }
}