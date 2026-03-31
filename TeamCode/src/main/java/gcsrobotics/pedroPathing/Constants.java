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
    // maxVelocity    — confirm units (in/s vs normalized 0-1)
    //                  forward velocity measured at 73.256 in/s
    // maxAcceleration — tune via forward zero power accel tuner
    //                  (31.504 recorded, confirm units match)
    // maxAngularVelocity / maxAngularAcceleration — tune via
    //                  lateral and turn tuners
    // =====================================================
    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
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
        public static final double OPEN_POSITION  = 0.4;
        public static final double CLOSE_POSITION = 1.0;
    }

    // ---- Hood (Axon MAX servo — alliance-independent) ----
    public static class Hood {
        public static final double CLOSE  = 0.10;
        public static final double MEDIUM = 0.25;
        public static final double TOP    = 0.40;
        public static final double FAR    = 0.55;
    }

    // ---- Flywheel (dual goBILDA 6000 RPM, shared shaft) ----
    // Velocities in rad/sec — use with setVelocity(v, AngleUnit.RADIANS)
    // PIDF starting values: tune F first on FTC Dashboard, then P
    // ⚠️  FAR (4200 RPM) likely exceeds motor max under load — confirm on hardware
    public static class Flywheel {
        public static final double PIDF_F = 11.7;
        public static final double PIDF_P = 0.5;
        public static final double PIDF_I = 0.0;
        public static final double PIDF_D = 0.0;

        // Target velocities in rad/sec
        public static final double VELOCITY_CLOSE  = 251.3;  // 2400 RPM
        public static final double VELOCITY_MEDIUM = 314.2;  // 3000 RPM
        public static final double VELOCITY_TOP    = 377.0;  // 3600 RPM
        public static final double VELOCITY_FAR    = 439.8;  // 4200 RPM ⚠️ TBD

        // Gate timing relative to flywheel readiness
        public static final double RPM_THRESHOLD       = 0.95; // 95% of target before firing
        public static final long   FLYWHEEL_TIMEOUT_MS = 3000;
        public static final long   SHOOT_DURATION_MS   = 2000; // TODO: tune on hardware
    }

    // ---- Intake ----
    public static class Intake {
        public static final double FORWARD_POWER = 1.0;
        public static final double REVERSE_POWER = -1.0;
    }
}