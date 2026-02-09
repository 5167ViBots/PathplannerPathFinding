// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.Console;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.GetPathPlannerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    public static RobotContainer ctr; 
    private final Field2d field;
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

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
           field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    }

    private void configureBindings() {
        ctr = this;
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withDeadband(.10)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // joystick.leftBumper().whileTrue()

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);


        double x = 80;
        double y = 20;
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());
        drivetrain.addVisionMeasurement(new Pose2d(x,y,new Rotation2d(0)),Timer.getFPGATimestamp());





        try { 
        joystick.y().whileTrue(new GetPathPlannerCommand());
        //joystick.y().whileTrue(AutoBuilder.buildAuto("auto"));

        }
        catch (Exception ex) { 
            System.out.println(ex.getMessage());
        }


        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation

        Pose2d targetPose = new Pose2d(0,1, Rotation2d.fromDegrees(0));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );


        joystick.povDown().whileTrue(pathfindingCommand);
    }





    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }                   
}
