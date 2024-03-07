package org.usfirst.frc4904.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    public final CANSparkMax motorLeft;
    public final CANSparkMax motorRight;

    public ClimberSubsystem(CANSparkMax motorLeft, CANSparkMax motorRight) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        this.motorLeft.setIdleMode(IdleMode.kBrake);
        this.motorRight.setIdleMode(IdleMode.kBrake);
    }

    public Command c_climberUp() {
        var cmd = this.run(() -> {
            this.motorLeft.setVoltage(1);
            this.motorRight.setVoltage(1);
        });
        cmd.setName("climber - c_climberUp");

        return cmd;
    }

    public Command c_climberDown() {
        var cmd = this.run(() -> {
            this.motorLeft.setVoltage(-1);
            this.motorRight.setVoltage(-1);
        });
        cmd.setName("climber - c_climberDown");

        return cmd;
    }

    public Command c_climberStop() {
        var cmd = this.run(() -> {
            this.motorLeft.setVoltage(0);
            this.motorRight.setVoltage(0);
        });
        cmd.setName("climber - c_climberDown");

        return cmd;
    }
}
