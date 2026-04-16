package gcsrobotics.control;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;


public abstract class OpModeBase extends LinearOpMode {
    public static volatile OpModeBase INSTANCE;
    public Follower follower;
    protected CommandRunner commandRunner;


    // goBILDA 6000 RPM Yellow Jacket — 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;
    public double currentTargetRPM = 0.0;


    // ---- Intake ----
    public DcMotorEx intakeMotor;


    // ---- Shooter ----
    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;
    public Servo hoodServo;
    public Servo gateServo;
    public CRServo kickstand;
    public Servo ledLight;


    // ---- Odometry ----
    public GoBildaPinpointDriver odo;


    // ---- Sensors ----
    // goBILDA laser sensors in digital/beam-break mode
    // getState() returns true = ball present (inverted from raw hardware — intentional)
    public DigitalChannel transferSensor;  // live — wired at transfer
    public DigitalChannel intakeSensor;    // live — wired at intake
    public DigitalChannel shotSensor;      // NOT YET WIRED — stubbed to false below


    protected abstract void initInternal();
    protected abstract void loopInternal();


    @Override
    public void runOpMode() {
        INSTANCE = this;


        // ── Pipe all telemetry to both Driver Station and FTC Dashboard ──
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );


        initHardware();
        follower = Constants.createFollower(hardwareMap);
        initInternal();


        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            loopInternal();
            updateFlywheel();
            telemetry.update();
        }
    }


    private void initHardware() {
        // ---- Odometry ----
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        // ---- Intake ----
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);


        // ---- Flywheels ----
        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorEx.Direction.REVERSE);


        // RUN_USING_ENCODER enables velocity PIDF control
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setVelocityPIDFCoefficients(
                Constants.Flywheel.PIDF_P,
                Constants.Flywheel.PIDF_I,
                Constants.Flywheel.PIDF_D,
                Constants.Flywheel.PIDF_F
        );
        flywheelRight.setVelocityPIDFCoefficients(
                Constants.Flywheel.PIDF_P,
                Constants.Flywheel.PIDF_I,
                Constants.Flywheel.PIDF_D,
                Constants.Flywheel.PIDF_F
        );


        // ---- Servos ----
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        kickstand = hardwareMap.get(CRServo.class, "kickstand");


        // ---- Sensors ----
        // All sensors: getState() = true means ball present (intentional inversion)
       //transferSensor = hardwareMap.get(DigitalChannel.class, "transferSensor");
        //transferSensor.setMode(DigitalChannel.Mode.INPUT);

        intakeSensor = hardwareMap.get(DigitalChannel.class, "intakeSensor");
        intakeSensor.setMode(DigitalChannel.Mode.INPUT);

        // shotSensor not yet wired — stub returns false until connected
        // TODO: uncomment when wired
        // shotSensor = hardwareMap.get(DigitalChannel.class, "shotSensor");
        // shotSensor.setMode(DigitalChannel.Mode.INPUT);


        // ---- LED ----
        // Initialized to OFF so servo output is defined from boot
        // If LED appears stuck ON, change OFF constant to 0.05 in Constants.LED
        ledLight = hardwareMap.get(Servo.class, "ledLight");
        ledLight.setPosition(Constants.LED.OFF);
    }


    // ---- Pose Helpers ----
    protected double getX()       { return follower.getPose().getX(); }
    protected double getY()       { return follower.getPose().getY(); }
    protected double getHeading() { return follower.getPose().getHeading(); }


    // ---- Flywheel Helpers (all in RPM) ----

    public void setFlywheelVelocity(double rpm) {
        currentTargetRPM = rpm;
    }


    // Three-zone control:
    //   Zone 1 (below threshold) — bang-bang: full power to recover quickly
    //   Zone 2 (above target by >2%) — coast: let RPM drop naturally
    //   Zone 3 (in range) — PIDF with voltage compensation for steady-state accuracy
    public void updateFlywheel() {
        if (currentTargetRPM <= 0) {
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
            return;
        }


        double actualRPM  = getFlywheelVelocity();
        double threshold  = currentTargetRPM * Constants.Flywheel.BANG_BANG_THRESHOLD;
        double upperBound = currentTargetRPM * 1.02;


        if (actualRPM < threshold) {
            // Zone 1 — below threshold, blast to recover quickly
            flywheelLeft.setPower(Constants.Flywheel.BANG_BANG_POWER);
            flywheelRight.setPower(Constants.Flywheel.BANG_BANG_POWER);
        } /* else if (actualRPM > upperBound) {
           // Zone 2 — above target, coast to let RPM drop naturally
           flywheelLeft.setPower(0);
           flywheelRight.setPower(0);
       } */ else {
            // Zone 3 — in range, PIDF with voltage compensation
            double voltage     = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double ticksPerSec = currentTargetRPM * TICKS_PER_REV / 60.0 * (12.0 / voltage);
            flywheelLeft.setVelocity(ticksPerSec);
            flywheelRight.setVelocity(ticksPerSec);
        }
    }


    public void setFlywheelPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }


    public double getFlywheelVelocity() {
        double avgTicks = (flywheelLeft.getVelocity() + flywheelRight.getVelocity()) / 2.0;
        return avgTicks * 60.0 / TICKS_PER_REV;
    }


    // ---- Sensor Helpers ----

    // Returns true when ball is present at transfer
    // getState() inversion is intentional — matches physical sensor behavior
    public boolean isBallAtTransfer() { return intakeSensor.getState(); }

    // Returns count of balls detected across all three sensors (0–3)
    // Same inversion logic as isBallAtTransfer() — getState() = true means ball present
    // shotSensor stubbed to false until wired — activate by wiring and uncommenting in initHardware()
    public int getBallCount() {
        int count = 0;
        if (intakeSensor.getState()) count++;
        if (intakeSensor.getState())   count++;
        // if (shotSensor.getState())  count++; // TODO: uncomment when shotSensor is wired
        return count;
    }

    // Updates LED color based on ball count:
    //   0 balls → OFF
    //   1 ball  → RED
    //   2 balls → YELLOW
    //   3 balls → GREEN
    public void updateLED() {
        switch (getBallCount()) {
            case 1:
                ledLight.setPosition(Constants.LED.SOLID_RED);
                break;
            case 2:
                ledLight.setPosition(Constants.LED.SOLID_YELLOW);
                break;
            case 3:
                ledLight.setPosition(Constants.LED.SOLID_GREEN);
                break;
            default:
                ledLight.setPosition(Constants.LED.OFF);
                break;
        }
    }
}

