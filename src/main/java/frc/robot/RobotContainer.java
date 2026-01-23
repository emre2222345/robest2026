// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.MathUtil;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    public double hedef = 0.0;
    public boolean basildi = false;
    public double hiz = 0.0;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.y().onTrue(Commands.runOnce(() -> 
        {
            hedef = 0.0;
            basildi = true;
        }));
        joystick.a().onTrue(Commands.runOnce(() ->
        {
            hedef = Math.PI;
            basildi = true;
        }));
        joystick.x().onTrue(Commands.runOnce(() ->
        {
            hedef = -Math.PI/2;
            basildi = true;
        }));
        joystick.b().onTrue(Commands.runOnce(() ->
        {
            hedef = Math.PI/2;
            basildi = true;
        }));
         drivetrain.setDefaultCommand(
			drivetrain.applyRequest(() -> 
            {
                if(Math.abs(joystick.getRightX()) > 0.1) 
                {
                    basildi = false;
                }   
                if(basildi)
                {
                    double hata = Rotation2d.fromRadians(hedef)
	                                        .minus(drivetrain.getState().Pose.getRotation())
	                                        .getRadians();
                    hata = Math.atan2(Math.sin(hata), Math.cos(hata));
                    if(Math.abs(hata) < Math.toRadians(3))
                    {
                        basildi = false;
                        hiz = 0;
                    }
                    else
                    {
                        	hiz = hata * 3;
	                        hiz = MathUtil.clamp(hiz, -MaxAngularRate, MaxAngularRate);
                    }
                }
                else
                {
                    hiz = -joystick.getRightX() * MaxAngularRate;
                }
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
					.withVelocityY(-joystick.getLeftX() * MaxSpeed)
					.withRotationalRate(hiz);
            }));
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
