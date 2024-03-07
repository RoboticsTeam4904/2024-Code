package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.humaninterface.drivers.SwerveGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.humaninput.Operator;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
        if (angle > 90) {
            return -.0001;
        }
        return .0001;
    }

    public Robot() {
        super();
    }

    @Override
    public void initialize() {
        delaySlider = Shuffleboard.getTab("Autonomous")
            .add("Delay", 0) // Default value is 0
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10)) // Set the min and max values for the slider
            .getEntry();
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
        //TODO create paths
        var command = new SequentialCommandGroup(
            new WaitCommand(delaySlider.getDouble(0)),
            RobotMap.Component.chassis.getAutonomousCommand("auton", true)
            // RobotMap.Component.arm.c_holdOuttakeAngle(-1, -1, null),
            // new WaitCommand(1),
            // RobotMap.Component.chassis.getAutonomousCommand("amp-leave_start", false),
            // RobotMap.Component.chassis.getAutonomousCommand("leave_start-return_start", false)
        );
        command.schedule();
        
    }

    @Override
    public void autonomousExecute() {}

    @Override
    public void disabledInitialize() {}

    @Override
    public void disabledExecute() {}

    @Override
    public void testInitialize() {}

    @Override
    public void testExecute() {}

    @Override
    public void alwaysExecute() {}
}
