// package org.usfirst.frc4904.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ClimberSubsystem extends SubsystemBase {
//     public final CANSparkMax motorLeft;
//     public final CANSparkMax motorRight;

//     public ClimberSubsystem(CANSparkMax motorLeft, CANSparkMax motorRight) {
//         this.motorLeft = motorLeft;
//         this.motorRight = motorRight;
//         this.motorLeft.setIdleMode(IdleMode.kBrake);
//         this.motorRight.setIdleMode(IdleMode.kBrake);
//         this.motorLeft.setInverted(true);
//     }

//     public Command c_climberUp() {
//         var cmd = this.run(() -> {
//             this.motorLeft.setVoltage(4);
//             this.motorRight.setVoltage(4);
//         });
//         cmd.setName("climber - c_climberUp");

//         return cmd;
//     }

//     public Command c_climberDown() {
//         var cmd = this.run(() -> {
//             this.motorLeft.setVoltage(-4);
//             this.motorRight.setVoltage(-4);
//         });
//         cmd.setName("climber - c_climberDown");

//         return cmd;
//     }
// }
