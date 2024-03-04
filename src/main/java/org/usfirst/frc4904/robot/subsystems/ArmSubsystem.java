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
    // TODO: get actual values
    public static final double kS = 0.00;
    public static final double kV = 1.3716;
    public static final double kA = 0.0299;
    public static final double kG = 0.4126;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    private static final double OUTTAKE_ANGLE = 90;
    private static final double INTAKE_ANGLE = 0;

    private static final double ARM_OFFSET = 186.14;

    public final CANTalonFX armMotor;
    public final ArmFeedforward feedforward;
    public final DutyCycleEncoder armEncoder;

    public ArmSubsystem(CANTalonFX armMotor, DutyCycleEncoder armEncoder) {
        this.armMotor = armMotor;
        this.armMotor.setBrakeOnNeutral();
        this.feedforward = new ArmFeedforward(kG, kS, kV, kA);
        this.armEncoder = armEncoder;
    }

    public double getCurrentAngleDegrees() {
        return armEncoder.getAbsolutePosition() * 360 - ARM_OFFSET;
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
