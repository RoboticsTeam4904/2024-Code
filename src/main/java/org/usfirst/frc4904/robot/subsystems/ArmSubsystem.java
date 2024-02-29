package org.usfirst.frc4904.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4904.standard.commands.CreateAndDisown;

import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.usfirst.frc4904.robot.Utils;

public class ArmSubsystem extends SubsystemBase {
    // TODO: get actual angles
    public static final double HARD_STOP_ARM_ANGLE = -38;
    public static final double HARD_STOP_BACK = 180 - HARD_STOP_ARM_ANGLE;

    // TODO: get actual angles
    public static final double INTAKE_ANGLE = -1;
    public static final double OUTTAKE_ANGLE = -1;

    // TODO: this is probably the wrong gearbox ratio
    public static final double GEARBOX_RATIO = (double) (48 * 60) / 26;
    public static final double DEGREES_PER_ROTATION = 360 / GEARBOX_RATIO;
    // TODO: do something with this or remove it idk
    public static final double GEARBOX_SLACK_DEGREES = 6;

    // TODO: get actual values (current values are from last year)
    public static final double kS = 0.10126;
    public static final double kV = 1.8894;
    public static final double kA = 0.048547;
    public static final double kG = 0.32;

    public static final double kP = 0.06;
    public static final double kI = 0.02;
    public static final double kD = 0;

    public final CANTalonFX armMotor;
    public final ArmFeedforward feedforward;
    public final DutyCycleEncoder armEncoder;

    public ArmSubsystem(CANTalonFX armMotor, DutyCycleEncoder armEncoder) {
        this.armMotor = armMotor;
        this.feedforward = new ArmFeedforward(kG, kS, kV, kA);
        this.armEncoder = armEncoder;
    }

    public double getCurrentAngleDegrees() {
        // TODO: add angle offset
        return armEncoder.getAbsolutePosition() * 360;
    }

    /**
     * [p]
     * Expects sensors to be zeroed at forward hard-stop.
     */

    public static double motorRevsToAngle(double revs) {
        return revs * DEGREES_PER_ROTATION;
    }

    public static double angleToMotorRevs(double angle) {
        return angle / DEGREES_PER_ROTATION;
    }

    public Command c_controlAngularVelocity(DoubleSupplier degPerSecDealer) {
        var cmd = this.run(() -> {
            var ff = this.feedforward.calculate(
                Units.degreesToRadians(getCurrentAngleDegrees()),
                Units.degreesToRadians(degPerSecDealer.getAsDouble())
            );
            SmartDashboard.putNumber("feedforward", ff);
            this.armMotor.setVoltage(ff);
        });
        
        cmd.setName("arm - c_controlAngularVelocity");
        
        return cmd;
    }

    public Command c_holdIntakeAngle(
        double maxVelDegPerSec,
        double maxAccelDegPerSecSquare,
        Supplier<Command> onArrivalCommandDealer
    ) {
        return c_holdRotation(INTAKE_ANGLE, maxVelDegPerSec, maxAccelDegPerSecSquare, onArrivalCommandDealer);
    }

    public Command c_holdOuttakeAngle(
        double maxVelDegPerSec,
        double maxAccelDegPerSecSquare,
        Supplier<Command> onArrivalCommandDealer
    ) {
        return c_holdRotation(OUTTAKE_ANGLE, maxVelDegPerSec, maxAccelDegPerSecSquare, onArrivalCommandDealer);
    }

    public Command c_holdRotation(
        double degreesFromHorizontal,
        double maxVelDegPerSec,
        double maxAccelDegPerSecSquare,
        Supplier<Command> onArrivalCommandDealer
    ) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocityDegPerSec) -> {
                double ff = this.feedforward.calculate(
                    Units.degreesToRadians(getCurrentAngleDegrees()),
                    Units.degreesToRadians(velocityDegPerSec)
                );
                SmartDashboard.putNumber("Intended voltage", maxAccelDegPerSecSquare);
                return ff;
            }
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelDegPerSec, maxAccelDegPerSecSquare)
        );

        var cmd = getEzMotion(
            controller,
            profile,
            new TrapezoidProfile.State(degreesFromHorizontal, 0),
            new TrapezoidProfile.State(getCurrentAngleDegrees(), 0)
        );

        return onArrivalCommandDealer == null
            ? cmd
            : Utils.nameCommand(
                "pivot w/ onArrival: " + cmd.getName(),
                new ParallelCommandGroup(
                    cmd,
                    new SequentialCommandGroup(
                        new WaitCommand(profile.totalTime()),
                        new CreateAndDisown("arm pivot", onArrivalCommandDealer)
                    )
                )
            );
    }

    private ezMotion getEzMotion(
        ezControl controller,
        TrapezoidProfile profile,
        TrapezoidProfile.State current,
        TrapezoidProfile.State goal
    ) {
        var cmd = new ezMotion(
            controller,
            this::getCurrentAngleDegrees,
            (double volts) -> {
                SmartDashboard.putNumber("Arm Volts", volts);
                this.armMotor.setVoltage(volts);
            },
            (double t) -> {
                TrapezoidProfile.State result = profile.calculate(t, current, goal);

                SmartDashboard.putNumber("deg setpoint", result.velocity);
                return new Pair<>(result.position, result.velocity);
            },
            this
        );

        cmd.setName("arm - c_holdRotation");
        return cmd;
    }
}
