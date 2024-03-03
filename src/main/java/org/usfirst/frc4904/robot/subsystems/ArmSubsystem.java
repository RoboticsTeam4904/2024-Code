package org.usfirst.frc4904.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc4904.standard.commands.CreateAndDisown;

import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.usfirst.frc4904.robot.Utils;

public class ArmSubsystem extends ProfiledPIDSubsystem {
    // TODO: get actual values
    public static final double kS = 0.00;
    public static final double kV = 2;
    public static final double kA = 0.1;
    public static final double kG = 0.3;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    private static final double OUTTAKE_ANGLE = 90;
    private static final double INTAKE_ANGLE = 0;

    private static final double ARM_OFFSET = 186.14;
    public enum ArmMode {
        VELOCITY , POSITION, DISABLED
    }
    private ArmMode armMode;
    private double radiansPerSecond;
    public final CANTalonFX armMotor;
    public final ArmFeedforward feedforward;
    public final DutyCycleEncoder armEncoder;

    public ArmSubsystem(CANTalonFX armMotor, DutyCycleEncoder armEncoder) {
        super(new ProfiledPIDController(kP, kI, kD, 
        new TrapezoidProfile.Constraints(Units.degreesToRadians(75), Units.degreesToRadians(75))));
        this.armMotor = armMotor;
        this.armMotor.setBrakeOnNeutral();
        this.feedforward = new ArmFeedforward(kG, kS, kV, kA);
        this.armEncoder = armEncoder;
        armMode = ArmMode.DISABLED;
    }
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the setpoint
        double calculatedFeedforward = feedforward.calculate(Units.degreesToRadians(getCurrentAngleDegrees()), setpoint.velocity, 0);
        // Add the feedforward to the PID output to get the motor output
        armMotor.setVoltage(output + calculatedFeedforward);
    }
    @Override
    public double getMeasurement() {
        return Units.degreesToRadians(getCurrentAngleDegrees());
    }

    public double getCurrentAngleDegrees() {
            return armEncoder.getAbsolutePosition() * 360 - ARM_OFFSET;
    }

    public void periodic() {
        // This method will be called once per scheduler run
        switch(armMode) {
            case POSITION:
                useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
                break;
            case VELOCITY:
                armMotor.setVoltage(feedforward.calculate(Units.degreesToRadians(getCurrentAngleDegrees()), this.radiansPerSecond, 0));
                break;
            case DISABLED:
                armMotor.stopMotor();
                break;
        };
    }
    public void setVelocity(Double degPerSecond) {
        this.armMode = ArmMode.VELOCITY;
        this.radiansPerSecond = Units.degreesToRadians(degPerSecond);
    }
    public void setArmAngleDegrees(double degrees) {
        this.armMode = ArmMode.POSITION;
        setGoal(Units.degreesToRadians(degrees));
    }
}
