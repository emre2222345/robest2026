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

//import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import pabeles.concurrency.IntOperatorTask.Max;
//import static edu.wpi.first.units.Units.*;

//import com.ctre.phoenix6.CANBus;
//import com.ctre.phoenix6.StatusCode;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.NeutralOut;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
//import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.MotorAlignmentValue;

//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.spark.SparkClosedLoopController;
//import edu.wpi.first.math.controller.ArmFeedforward;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathConstraints;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;   

public class RobotContainer {
    public boolean armStatus = false;
    public boolean intakeStatus = false;
    public boolean shooterStatus = false;
    private double targetRPS = 60;;
    private Command ShootlaCopy;
    private TalonFX intakeMotor1 = new TalonFX(30);
    private TalonFX intakeMotor2 = new TalonFX(31);
    private SparkMax armMotor = new SparkMax(36, MotorType.kBrushless);
    private SparkMax FeederMotor = new SparkMax(32, MotorType.kBrushless);
    private TalonFX shooterMotor1 = new TalonFX(33);
    private TalonFX shooterMotor2 = new TalonFX(34);
    private TalonFX shooterFeeder = new TalonFX(35);
        
    private SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private final double kP_Translation = 1.5; 
    private final double kP_Rotation = 1.5;
    private Pose2d kTargetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    private boolean drivingToPose = false;

    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double hizCarpan = 1;
    private double lastPrintTime = 0.0;

    //private boolean snap = false; // snaps to 45 degree angles

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

    private final VelocityVoltage m_motorVelocityVoltage1 = new VelocityVoltage(0);

    public RobotContainer() {
        var pidmain = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        pidmain.Slot0.kP = 0.75;
        pidmain.Slot0.kD = 0.075;

        pidmain.CurrentLimits.SupplyCurrentLimit = 35;
        pidmain.CurrentLimits.SupplyCurrentLimitEnable = true;
        pidmain.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        pidmain.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMotor1.getConfigurator().apply(pidmain);
        intakeMotor1.getConfigurator().apply(pidmain);
        shooterFeeder.getConfigurator().apply(pidmain);
                
        pidmain.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2.getConfigurator().apply(pidmain);        
        intakeMotor2.getConfigurator().apply(pidmain);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35);
        config.inverted(false);
        config.closedLoop.pid(0.75, 0, 0.075);
        config.idleMode(IdleMode.kBrake);

        FeederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.smartCurrentLimit(40);
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(false);
        armConfig.encoder.positionConversionFactor(360.0 / 120.0);

        armConfig.closedLoop
            .pid(0.02, 0.0, 0.0)
            .outputRange(-1, 1);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.getEncoder().setPosition(0);
        
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

        /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/
        joystick.x().onTrue(Commands.runOnce(() -> {
            if (intakeStatus) {
                intakeStatus = false;
                intakeMotor1.set(0.0);
                intakeMotor2.set(0);
            } else {
                intakeStatus = true;
                intakeMotor1.set(0.7);
                intakeMotor2.set(0.7);
            }
        }));

        joystick.b().onTrue(Commands.runOnce(() -> {
        if (shooterStatus) {
            shooterStatus = false;
            if(ShootlaCopy != null) ShootlaCopy.cancel();
            shooterFeeder.set(0);
            shooterMotor1.set(0);
            shooterMotor2.set(0);
            FeederMotor.set(0);
        } else {
            shooterStatus = true;
            targetRPS = ((double)51.0); 
            ShootlaCopy = Shootla(targetRPS);
            ShootlaCopy.schedule();
        }
        }));

        joystick.y().onTrue(Commands.runOnce(() -> {
            if(armStatus) armController.setReference(0.0, ControlType.kPosition);
            else armController.setReference(90.0, ControlType.kPosition);
            armStatus = !armStatus;
        }));

        //joystick.povUp().onTrue(new RunCommand(()->SmartDashboard.putNumber("Shooter1Rpm", shooterMotor1.getEncoder().getVelocity())));
        joystick.povRight().onTrue(Commands.runOnce(()->{kTargetPose = drivetrain.getState().Pose;},drivetrain));
        joystick.povDown().onTrue(Commands.runOnce(() ->{
            if(drivingToPose)  drivingToPose = false;   
            else driveToPoseCommand().schedule();
        }));

        joystick.povLeft().onTrue(Commands.runOnce(() -> {
            LimelightHelpers.SetRobotOrientation("limelight",
                drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            if (mt2 != null && mt2.tagCount > 0) {
                drivetrain.resetPose(mt2.pose);
                System.out.printf(
                    "[Limelight] Pose RESET (MT2) -> X: %.2f m, Y: %.2f m, Yaw: %.2f°%n",
                    mt2.pose.getX(), mt2.pose.getY(), mt2.pose.getRotation().getDegrees()
                );
            } else {
                System.out.println("Tag Error.");
            }
        }));

        drivetrain.registerTelemetry((state) -> {
            logger.telemeterize(state);
            LimelightHelpers.SetRobotOrientation(
                "limelight",
                state.Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
            );
            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            boolean rejectUpdate = false;
            if (Math.abs(state.Speeds.omegaRadiansPerSecond) > Math.toRadians(720)) {
                rejectUpdate = true;
            }
            if (mt2 == null || mt2.tagCount == 0) {
                rejectUpdate = true;
            }
            if (!rejectUpdate) {
                double xyStdDev = 0.5 * (1.0 + mt2.avgTagDist * 0.1) / Math.max(1, mt2.tagCount);
                drivetrain.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
                );
                drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                double now = Timer.getFPGATimestamp();
                if (now - lastPrintTime >= 1.0) {
                    lastPrintTime = now;
                    System.out.printf(
                        "[Limelight] -> X: %.2f m, Y: %.2f m, Yaw: %.2f°, TagCount: %d, AvgDist: %.2f m%n",
                        mt2.pose.getX(),
                        mt2.pose.getY(),
                        mt2.pose.getRotation().getDegrees(),
                        mt2.tagCount,
                        mt2.avgTagDist
                    );
                }
            }
        });
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
            
            return (!drivingToPose || (dist < 0.05 && degError < 2.0));
        })
        .finallyDo((interrupted) -> drivingToPose = false); 
    }

    public Command Shootla(double targetRPS) {
    return Commands.run(() -> {
        shooterMotor1.setControl(m_motorVelocityVoltage1.withVelocity(targetRPS));
        shooterMotor2.setControl(m_motorVelocityVoltage1.withVelocity(targetRPS));
        if(shooterMotor1.getVelocity().getValueAsDouble() >= targetRPS * 0.85 && shooterStatus){
            shooterFeeder.setControl(m_motorVelocityVoltage1.withVelocity(targetRPS));
            FeederMotor.set((0.3));
        }
        else if (!shooterStatus){
            shooterFeeder.set(0);
            FeederMotor.set(0);
        }
    }).until(() -> !shooterStatus);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }


}