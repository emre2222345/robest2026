// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import pabeles.concurrency.IntOperatorTask.Max;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
    public boolean intakeStatus = false;
    public boolean shooterStatus = false;
    private SparkMax intakeMotor  = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
    private SparkMax shooterMotor1  = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
    private SparkMax shooterMotor2  = new SparkMax(32, SparkLowLevel.MotorType.kBrushless);
    private SparkMax shooterMotor3  = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);
    private SparkMax shooterMotor4  = new SparkMax(34, SparkLowLevel.MotorType.kBrushless);
    private SparkMaxConfig shooterConfig = new SparkMaxConfig();

    private final double kP_Translation = 1.5; 
    private final double kP_Rotation = 1.5;
    private Pose2d kTargetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    private boolean drivingToPose = false;

    private double MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private boolean snap = false; // snaps to 45 degree angles

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withMaxAbsRotationalRate(MaxAngularRate);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        ClosedLoopConfig pid = shooterConfig.closedLoop;
        pid.p(0.00032);
        pid.i(0);
        pid.d(0.000032);
        pid.velocityFF(0.00017);

        shooterConfig.smartCurrentLimit(35);

        shooterConfig.inverted(true);
        shooterMotor1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor4.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterConfig.inverted(false);
        shooterMotor2.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor3.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    var x = joystick.getRightX();
                    var y = joystick.getRightY();
                    if(Math.abs(x) > 0.4 || Math.abs(y) > 0.4) {
                        var rotation = new Rotation2d(-y, -x);
                        if(snap) rotation = new Rotation2d(Math.toRadians(Math.round(rotation.getDegrees() / 45.0) * 45));
                        return driveFacing.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                .withTargetDirection(rotation);
                    } else {
                        return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                .withRotationalRate(0);
                    }
                }
            ).unless(() -> drivingToPose)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.y().onTrue(Commands.runOnce(() -> {
            if (intakeStatus) {
                intakeStatus = false;
                intakeMotor.set(0.0);
            } else {
                intakeStatus = true;
                intakeMotor.set(1.0);
            }
        }));

        joystick.x().onTrue(Commands.runOnce(() -> {
        if (shooterStatus) {
            shooterStatus = false;
            shooterMotor1.getClosedLoopController().setReference(0, ControlType.kVelocity);
            shooterMotor2.getClosedLoopController().setReference(0, ControlType.kVelocity);
            shooterMotor3.getClosedLoopController().setReference(0, ControlType.kVelocity);
            shooterMotor4.getClosedLoopController().setReference(0, ControlType.kVelocity);
        } else {
            shooterStatus = true;
            double targetRPM = 3400; 
            shooterMotor1.getClosedLoopController().setReference(targetRPM, ControlType.kVelocity);
            shooterMotor2.getClosedLoopController().setReference(targetRPM, ControlType.kVelocity);
            shooterMotor3.getClosedLoopController().setReference(targetRPM, ControlType.kVelocity);
            shooterMotor4.getClosedLoopController().setReference(targetRPM, ControlType.kVelocity);
        }
        }));

        joystick.povUp().onTrue(new RunCommand(()->SmartDashboard.putNumber("Shooter1Rpm", shooterMotor1.getEncoder().getVelocity())));
        joystick.povRight().onTrue(Commands.runOnce(()->{kTargetPose = drivetrain.getState().Pose;},drivetrain));
        joystick.povDown().onTrue(
            Commands.either(
                Commands.runOnce(() -> drivingToPose = false), 
                driveToPoseCommand(), 
                () -> drivingToPose
            )
        );

        joystick.rightStick().onTrue(Commands.runOnce(() -> snap = !snap));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

        public Command driveToPoseCommand() {
        return drivetrain.applyRequest(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;

            double xError = kTargetPose.getX() - currentPose.getX();
            double yError = kTargetPose.getY() - currentPose.getY();
            
            double angleError = edu.wpi.first.math.MathUtil.inputModulus(
                kTargetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(),
                -Math.PI, 
                Math.PI
            );

            double xVel = xError * kP_Translation;
            double yVel = yError * kP_Translation;
            double rotVel = angleError * kP_Rotation;

            if(xVel > 0) xVel = Math.min(xVel, MaxSpeed);
            else xVel = Math.max(xVel, -MaxSpeed);

            if(yVel > 0) yVel = Math.min(yVel, MaxSpeed);
            else yVel = Math.max(yVel, -MaxSpeed);
            
            if(rotVel > 0) rotVel = Math.min(rotVel, MaxAngularRate);
            else rotVel = Math.max(rotVel, -MaxAngularRate);

            return drive.withVelocityX(xVel)
                        .withVelocityY(yVel)
                        .withRotationalRate(rotVel);
        })
        .beforeStarting(() -> drivingToPose = true)
        .until(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;
            double dist = currentPose.getTranslation().getDistance(kTargetPose.getTranslation());
            double degError = Math.abs(currentPose.getRotation().minus(kTargetPose.getRotation()).getDegrees());
            
            return dist < 0.05 && degError < 2.0;
        })
        .finallyDo((interrupted) -> drivingToPose = false); 
    }

    public Command getAutonomousCommand() {
    Command seedHeading = drivetrain.runOnce(() -> drivetrain.seedFieldCentric(new Rotation2d()));
    try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        Command pathCommand = AutoBuilder.followPath(path);
        return new SequentialCommandGroup(
            seedHeading,
            pathCommand
        );
    } catch (Exception e) {
        return seedHeading;
    }
}


}