// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import pabeles.concurrency.IntOperatorTask.Max;

//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.controls.VelocityVoltage;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
    public boolean intakeStatus = false;
    public boolean shooterStatus = false;
    private SparkMax intakeMotor  = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
    private TalonFX shooterMotor1 = new TalonFX(31);
    private TalonFX shooterMotor2 = new TalonFX(32);

    private final double kP_Translation = 1.5; 
    private final double kP_Rotation = 1.5;
    private Pose2d kTargetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    private boolean drivingToPose = false;

    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private boolean slowMode = false;
    private double hizCarpan = 1;

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
        var pid = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        pid.Slot0.kP = 0.00032;
        pid.Slot0.kI = 0.0;
        pid.Slot0.kD = 0.000032;
        pid.Slot0.kV = 0.00017;

        pid.CurrentLimits.SupplyCurrentLimit = 35;
        pid.CurrentLimits.SupplyCurrentLimitEnable = true;
        pid.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pid.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMotor1.getConfigurator().apply(pid);
        
        pid.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2.getConfigurator().apply(pid);
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * hizCarpan) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed *hizCarpan) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate *hizCarpan) // Drive counterclockwise with negative X (left)
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
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        } else {
            shooterStatus = true;
            double targetRPS = (3400.0/60.0); 
            shooterMotor1.setControl(new VelocityVoltage(targetRPS));
            shooterMotor2.setControl(new VelocityVoltage(targetRPS));

        }
        }));

        //joystick.povUp().onTrue(new RunCommand(()->SmartDashboard.putNumber("Shooter1Rpm", shooterMotor1.getEncoder().getVelocity())));
        joystick.povUp().onTrue(Commands.runOnce(()->{
            if(!slowMode){
                hizCarpan = 0.4;
                slowMode = true;
            }
            else{
                hizCarpan = 1;
                slowMode = false;
            }
        }));
        joystick.povRight().onTrue(Commands.runOnce(()->{kTargetPose = drivetrain.getState().Pose;},drivetrain));
        joystick.povDown().onTrue(Commands.runOnce(() ->{
            if(drivingToPose)  drivingToPose = false;   
            else driveToPoseCommand().schedule();
        }));
        joystick.povLeft().onTrue(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d()))));

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
            
            return dist < 0.05 && degError < 2.0 && drivingToPose;
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