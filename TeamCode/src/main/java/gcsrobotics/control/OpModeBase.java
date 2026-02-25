package gcsrobotics.control;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;

public abstract class OpModeBase extends LinearOpMode {
    public static volatile OpModeBase INSTANCE;
    public Follower follower;
    protected CommandRunner commandRunner;

    protected abstract void initInternal();
    protected abstract void loopInternal();

    /*eg.*/ public DcMotorEx flywheel;
    /*eg.*/ public DcMotorEx intake;

    @Override
    public void runOpMode() {

        /* eg.*/ flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        /* eg.*/ intake = hardwareMap.get(DcMotorEx.class, "intake");

        follower = Constants.createFollower(hardwareMap);
        initInternal();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            loopInternal();
        }
    }

}
