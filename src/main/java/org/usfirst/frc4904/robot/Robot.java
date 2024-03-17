package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.humaninput.Operator;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static org.usfirst.frc4904.robot.Utils.nameCommand;

import java.util.Map;

import javax.swing.RowFilter.ComparisonType;

import org.usfirst.frc4904.robot.RobotMap.Component;

public class Robot extends CommandRobotBase {
    private final Driver driver = new SwerveGain();
    private final Operator operator = new DefaultOperator();
    private final RobotMap map = new RobotMap();
    private final CustomCommandJoystick joystick = new CustomCommandJoystick(0, 0.1);
    private GenericEntry delaySlider;

    // public double armJoystick(double input) {

    //     return input*30 - Math.signum(input);
    // }
    protected double scaleGain(double input, double gain, double exp) {
        return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
    }
    protected double nudge(double angle) {
        if (angle > 90 && angle < 180) {
            if (angle > 100 &&  (RobotMap.HumanInput.Operator.joystick.getAxis(1) * -120) > 0) {
                return -.0001 - (RobotMap.HumanInput.Operator.joystick.getAxis(1) * -120);
            }
            return -.0001;
        }
        if (angle < 2 && angle > 0 && (RobotMap.HumanInput.Operator.joystick.getAxis(1) * -120) < 0) {
            return .0001 - (RobotMap.HumanInput.Operator.joystick.getAxis(1) * -120);
        }
        return .0001;
    }

    public Robot() {
        super();
    }

    @Override
    public void initialize() {
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void teleopInitialize() {
        driver.bindCommands();
        operator.bindCommands();

        RobotMap.Component.chassis.setDefaultCommand(
            RobotMap.Component.chassis.driveCommand(
                () -> driver.getY(),
                () -> driver.getX(),
                () -> driver.getTurnSpeed()
            )
        );
        RobotMap.Component.arm.setDefaultCommand(
            RobotMap.Component.arm.c_controlAngularVelocity(() -> RobotMap.HumanInput.Operator.joystick.getAxis(1) * -120 + nudge(RobotMap.Component.arm.getCurrentAngleDegrees()))
        );
    }

    @Override
    public void teleopExecute() {
        SmartDashboard.putBoolean("button", RobotMap.HumanInput.Driver.turnJoystick.button1.getAsBoolean());
        SmartDashboard.putNumber("max angular velocity", RobotMap.Component.chassis.swerveDrive.getMaximumAngularVelocity());
        SmartDashboard.putNumber("arm angle", RobotMap.Component.arm.getCurrentAngleDegrees());
        SmartDashboard.putNumber("arm voltage", RobotMap.Component.armMotor.getMotorVoltage().getValue());
        SmartDashboard.putNumber("joystick position", RobotMap.HumanInput.Operator.joystick.getAxis(1) * 30);
        // RobotMap.Component.armMotor.setVoltage(2);
    }

    @Override
    public void autonomousInitialize() {
        RobotMap.Component.arm.setDefaultCommand(null);
        RobotMap.Component.chassis.zeroGyro();
        RobotMap.getAutonomousCommand().schedule();

        // RobotMap.Component.arm.scuffed().schedule();

        
        // //TODO create paths
        // var command = new SequentialCommandGroup(
        //     new WaitCommand(0),
        //     // RobotMap.Component.arm.c_holdOuttakeAngle(-1, -1, null),
        //     // new WaitCommand(1),
        //     // RobotMap.Component.chassis.getAutonomousCommand("amp-leave_start", false),
        //     // RobotMap.Component.chassis.getAutonomousCommand("leave_start-return_start", false)
        // );
        // RobotMap.Component.chassis.driveToPose(new Pose2d(1, 1, new Rotation2d(0))).schedule();

        // RobotMap.Component.chassis.getAutonomousCommand("auton", true).schedule();

        // command.schedule();
        //manual no-pathplanner "autons"
        //RobotMap.Component.chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        // new SequentialCommandGroup(
        //     new ParallelRaceGroup(RobotMap.Component.chassis.driveCommand(() -> .25, () -> 0, () -> 0), new WaitUntilCommand(() -> RobotMap.Component.chassis.getPose().getX() > .25)),
        //     new InstantCommand(() -> System.out.print("drove forward" + RobotMap.Component.chassis.getPose().getX())),
        //     new ParallelRaceGroup(RobotMap.Component.chassis.driveCommand(() -> 0, () -> 0.25, () -> 0), new WaitUntilCommand(() -> RobotMap.Component.chassis.getPose().getY() > .3)),
        //     new InstantCommand(() -> System.out.print("drove right"  + RobotMap.Component.chassis.getPose().getY())),
        //     RobotMap.Component.arm.scuffed(50, 50, null),
        //     new InstantCommand(() -> System.out.print("arm up"  + RobotMap.Component.arm.getCurrentAngleDegrees())),
        //     new ParallelCommandGroup(
        //         new ParallelRaceGroup(RobotMap.Component.chassis.driveCommand(() -> 0, () -> 0.25, () -> 0), new WaitUntilCommand(() -> RobotMap.Component.chassis.getPose().getY() > 1)),
        //         RobotMap.Component.arm.scuffedback()
        //     ),
        //     new InstantCommand(() -> System.out.print("arm down and clear"  + RobotMap.Component.arm.getCurrentAngleDegrees()))
        // ).schedule();
        // new SequentialCommandGroup(
        //     new ParallelRaceGroup(RobotMap.Component.chassis.driveCommand(() -> .25, () -> 0.3, () -> 0), new WaitUntilCommand(() -> RobotMap.Component.chassis.getPose().getX() > .25)),
        //     new InstantCommand(() -> System.out.print("drove diaganol" + RobotMap.Component.chassis.getPose().getX())),
        //     RobotMap.Component.arm.scuffed(50, 50, null),
        //     new InstantCommand(() -> System.out.print("arm up"  + RobotMap.Component.arm.getCurrentAngleDegrees())),
        //     new ParallelCommandGroup(
        //         new ParallelRaceGroup(RobotMap.Component.chassis.driveCommand(() -> 0, () -> 0.25, () -> 0), new WaitUntilCommand(() -> RobotMap.Component.chassis.getPose().getY() > 1)),
        //         RobotMap.Component.arm.scuffedback()
        //     ),
        //     new InstantCommand(() -> System.out.print("arm down and clear"  + RobotMap.Component.arm.getCurrentAngleDegrees()))
        // ).schedule();
        // new ParallelRaceGroup(RobotMap.Component.chassis.driveCommand(() -> .25, () -> 0, () -> 0), new WaitUntilCommand(() -> RobotMap.Component.chassis.getPose().getX() > 1.5)).schedule();
    }

    @Override
    public void autonomousExecute() {}

    @Override
    public void disabledInitialize() {}

    @Override
    public void disabledExecute() {}

    @Override
    public void testInitialize() {
        //RobotMap.Component.arm.scuffed(50, 50, null).schedule();
        // RobotMap.Component.chassis.setDefaultCommand(null);

    }

    @Override
    public void testExecute() {}

    @Override
    public void alwaysExecute() {}
}
