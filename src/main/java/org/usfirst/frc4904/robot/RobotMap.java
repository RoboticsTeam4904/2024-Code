package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.subsystems.ArmSubsystem;
import org.usfirst.frc4904.robot.subsystems.ClimberSubsystem;
// import org.usfirst.frc4904.robot.subsystems.ClimberSubsystem;
import org.usfirst.frc4904.robot.subsystems.SwerveSubsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;
            public static final int joystick = 2;
        }

        // 2023 robot constants
        // TODO: update ports for swerve
        public static class CANMotor {
            public static final int FRONT_LEFT_DRIVE = 1;
            public static final int FRONT_LEFT_TURN = 5;
            public static final int FRONT_RIGHT_DRIVE = 2;
            public static final int FRONT_RIGHT_TURN = 6;
            public static final int BACK_LEFT_DRIVE = 3;
            public static final int BACK_LEFT_TURN = 7;
            public static final int BACK_RIGHT_DRIVE = 4;
            public static final int BACK_RIGHT_TURN = 8;
            public static final int CLIMBER_LEFT = 11;
            public static final int CLIMBER_RIGHT = 12;
            public static final int ARM_MOTOR = 9;
        }

        public static class PWM {
            public static final int ARM_ENCODER = 0;
        }

        public static class CAN {
        }

        public static class Pneumatics {
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        // 2023 robot constants
        public static class Chassis {
            // 5.1:1 gear ratio
            // 46.42:1 gear ratio
            // 3 inch wheels
            // +/- 0.5 inches
            // +/- 0.5 inches
            // no offset

            public static final double GEAR_RATIO_DRIVE = 5.1;
            public static final double GEAR_RATIO_TURN = 46.42;
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(12.241*2);
            public static final double TRACK_LENGTH_METERS = Units.inchesToMeters(12.259*2);
            public static final double CHASSIS_LENGTH = Units.inchesToMeters(28);
            public static final Translation2d CENTER_MASS_OFFSET = new Translation2d(0,0);
            public static final double EncoderTicksPerRevolution = 2048;

            // allowed max speed and acceleration in m/s
            public static final double MAX_SPEED = 4.8;
            public static final double MAX_ACCELERATION = 3;

            public static final double MAX_TRANSLATION_SPEED = 4.8;

            // allowed max turn and acceleration in deg/s
            public static final double MAX_TURN_SPEED = 360;
            public static final double MAX_TURN_ACCELERATION = 180;
        }
    }

    public static class PID {
        public static class Drive {
            // TODO: tune, from maxswerve but seem too low
            public static final double kP = .04;
            public static final double kI = 0;
            public static final double kD = 0;

            // pre-sfr on-carpet characterization
            // public static final double kS = 0.025236;
            // public static final double kV = 3.0683;
            // public static final double kA = 0.7358;

            // post sfr characterization
            public static final double kS = .02;
            public static final double kV = 3;
            public static final double kA = .5;
        }

        public static class Turn {
            // pid constants
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;

            // TODO: find actual values using sysid
            // feedforward constants
            public static final double kS = .02;
            public static final double kV = 3;
            public static final double kA = .5;
        }
    }

    public static class Component {
        // TODO: turn motors are not falcons, so can't use cantalons
        public static CANTalonFX FLdrive;
        public static CustomCANSparkMax FLturn;
        public static CANTalonFX FRdrive;
        public static CustomCANSparkMax FRturn;
        public static CANTalonFX BLdrive;
        public static CustomCANSparkMax BLturn;
        public static CANTalonFX BRdrive;
        public static CustomCANSparkMax BRturn;

        // encoders are dutycycle, not standard can
        public static DutyCycleEncoder FLturnEncoder;
        public static DutyCycleEncoder FRturnEncoder;
        public static DutyCycleEncoder BLturnEncoder;
        public static DutyCycleEncoder BRturnEncoder;

        public static AHRS navx;

        public static SwerveSubsystem chassis;

        public static CANTalonFX armMotor;
        public static DutyCycleEncoder armEncoder;
        public static ArmSubsystem arm;
        public static ClimberSubsystem climber;
        public static CANSparkMax climberLeft;
        public static CANSparkMax climberRight;
    }

    public static class NetworkTables {
        public static NetworkTableInstance instance;

        public static class Odometry {
            public static NetworkTable table;
            public static NetworkTableEntry pose;
            public static NetworkTableEntry accel;
            public static NetworkTableEntry turretAngle;
        }

        public static class Localization {
            public static NetworkTable table;
            public static NetworkTableEntry goalDistance;
            public static NetworkTableEntry goalRelativeAngle;
        }
    }

    public static class Input {}

    public static class HumanInput {
        public static class Driver {
            public static CustomCommandXbox xbox;
            public static CustomCommandJoystick xyJoystick;
            public static CustomCommandJoystick turnJoystick;
        }

        public static class Operator {
            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.chassis = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), 360, .0473, 4.5);
        Component.armMotor = new CANTalonFX(Port.CANMotor.ARM_MOTOR);
        Component.armEncoder = new DutyCycleEncoder(Port.PWM.ARM_ENCODER); // TODO: fix port
        Component.arm = new ArmSubsystem(Component.armMotor, Component.armEncoder);
        NamedCommands.registerCommand("armUp", Component.arm.scuffed());
        Component.climberRight = new CANSparkMax(Port.CANMotor.CLIMBER_RIGHT, MotorType.kBrushless);
        Component.climberLeft = new CANSparkMax(Port.CANMotor.CLIMBER_LEFT, MotorType.kBrushless);
        Component.climber = new ClimberSubsystem(Component.climberLeft, Component.climberRight);

        HumanInput.Driver.xyJoystick = new CustomCommandJoystick(Port.HumanInput.xyJoystickPort, 0.01);
        HumanInput.Driver.turnJoystick = new CustomCommandJoystick(Port.HumanInput.zJoystickPort, 0.01);
        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
    }
}
