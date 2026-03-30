package gcsrobotics.commands;

import com.qualcomm.robotcore.hardware.CRServo;
import gcsrobotics.pedroPathing.Constants;

public class KickstandSubsystem {

    private final CRServo kickstandServo;

    public KickstandSubsystem(CRServo kickstandServo) {
        this.kickstandServo = kickstandServo;
    }

    public void deploy() {
        kickstandServo.setPower(Constants.Kickstand.DEPLOY_POWER);
    }

    public void retract() {
        kickstandServo.setPower(Constants.Kickstand.RETRACT_POWER);
    }

    public void stop() {
        kickstandServo.setPower(0.0);
    }
}