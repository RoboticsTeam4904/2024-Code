package org.usfirst.frc4904.robot.humaninterface.drivers;
import org.usfirst.frc4904.standard.humaninput.Driver;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;

public class SwerveGain extends Driver {
    // TODO: tune
    double SPEED_EXP = 2;
    double TURN_EXP = 2;

    static boolean button1 = false;

    public SwerveGain() {
        super("SwerveGain");
    }

    protected double scaleGain(double input, double exp) {
        return Math.pow(Math.abs(input), exp) * Math.signum(input);
    }

    public void bindCommands() {
        RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(
            new InstantCommand(() -> RobotMap.Component.chassis.brickMode())
        );
    }

    public double getX() {
        double raw = RobotMap.HumanInput.Driver.xyJoystick.getX();
        return scaleGain(raw, SPEED_EXP);
    }

    public double getY() {
        double raw = RobotMap.HumanInput.Driver.xyJoystick.getY();
        return scaleGain(raw, SPEED_EXP);
    }

    public double getTurnSpeed() {
        double raw = RobotMap.HumanInput.Driver.turnJoystick.getX();
        return scaleGain(raw, TURN_EXP);
    }

    public boolean getButton1Pressed() {
        RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(
            new InstantCommand(() -> SwerveGain.button1 = true)
        );

        RobotMap.HumanInput.Driver.turnJoystick.button1.onFalse(
            new InstantCommand(() -> SwerveGain.button1 = false)
        );

        return SwerveGain.button1;
    }
}
