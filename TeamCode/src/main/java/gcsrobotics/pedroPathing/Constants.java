package gcsrobotics.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // =====================================================
    // TODO: Populate after Pedro Pathing tuning is complete.
    // =====================================================
    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    // =====================================================
    // ROBOT SUBSYSTEM CONSTANTS
    // =====================================================

    // ---- Kickstand (CRServo — runs while button held) ----
    public static class Kickstand {
        public static final double DEPLOY_POWER  =  1.0;
        public static final double RETRACT_POWER = -1.0;
        public static final String MOTOR_NAME    = "kickstand";
    }

    // ---- Gate (Standard Servo) ----
    public static class Gate {
        public static final double OPEN_POSITION  = 0.6;
        public static final double CLOSE_POSITION = 0.0;
    }

    // ---- Hood (Axon MAX servo — alliance-independent) ----
    // A = CLOSE, X = MEDIUM, Y = TOP, B = FAR
    public static class Hood {
        public static final double CLOSE  = 1.0; //a
        public static final double MEDIUM = 0.9; //x
        public static final double TOP    = 0.8; //y
        public static final double FAR    = 0.7; //b
    }

    // ---- Flywheel (dual goBILDA 6000 RPM, shared shaft) ----
    // All velocities in RPM — converted to ticks/sec in OpModeBase
    // 28 ticks/rev for goBILDA Yellow Jacket
    // PIDF: tune F first on FTC Dashboard for flat steady-state, then P for ramp
    public static class Flywheel {
        public static final double PIDF_F = 11.7;
        public static final double PIDF_P = 0.5;
        public static final double PIDF_I = 0.0;
        public static final double PIDF_D = 0.0;

        // Target velocities in RPM — A=CLOSE, X=MEDIUM, Y=TOP, B=FAR
        // TODO: tune all four values on hardware
        public static final double VELOCITY_CLOSE  = 2900;  // A — slowest
        public static final double VELOCITY_MEDIUM = 3200;  // X
        public static final double VELOCITY_TOP    = 3500;  // Y
        public static final double VELOCITY_FAR    = 4400;  // B — fastest

        public static final double RPM_THRESHOLD       = 0.95;
        public static final long   FLYWHEEL_TIMEOUT_MS = 3000;
        public static final long   SHOOT_DURATION_MS   = 2000;
    }

    // ---- Intake ----
    public static class Intake {
        public static final double FORWARD_POWER = -1.0;
        public static final double REVERSE_POWER = 1.0;
    }
}