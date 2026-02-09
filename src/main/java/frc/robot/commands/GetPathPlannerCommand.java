package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class GetPathPlannerCommand extends Command {
    Command pathfindcommand;

    public GetPathPlannerCommand() {
        addRequirements(RobotContainer.ctr.drivetrain);
    }

    public Pose2d GetCurrentPose() {
        return RobotContainer.ctr.drivetrain.getState().Pose;
    }

    public ChassisSpeeds GetCurrentSpeeds() {
        return RobotContainer.ctr.drivetrain.getState().Speeds;
    }

    @Override
    public void initialize()  {

        Pose2d targetPose = new Pose2d();
        PathConstraints constraints = new PathConstraints(1, 1, 5, 5, 6);
        double goalEndVel = 0;
        Supplier<Pose2d> poseSupplier = () -> {return GetCurrentPose();};
        Supplier<ChassisSpeeds> speedsSupplier = () -> {return GetCurrentSpeeds();};
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
        PathFollowingController controller = new PPHolonomicDriveController(new PIDConstants(10), new PIDConstants(7));
        RobotConfig robotConfig;
         try {
             robotConfig = RobotConfig.fromGUISettings();
         } catch (Exception e) {
             System.out.println(e.getMessage());
             return;
         }

        output = (speeds, feedforwards) -> RobotContainer.ctr.drivetrain.setControl(
                            RobotContainer.ctr.drivetrain.m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));



        pathfindcommand = new PathfindingCommand(targetPose,
                constraints,
                goalEndVel,
                poseSupplier,
                speedsSupplier,
                output,
                controller,
                robotConfig);

        // Pose2d nullposition = GetCurrentPose();
        // nullposition = new Pose2d(15,15,new Rotation2d(45));
        // //nullposition = nullposition.rotateBy(Rotation2d.fromDegrees(45));
        // // PathConstraints constraints = new PathConstraints(1,1,5,5);
        // PathConstraints constraints = new PathConstraints(
        // 3.0, 4.0,
        // Units.degreesToRadians(540), Units.degreesToRadians(720));

        // pathfindcommand = AutoBuilder.pathfindToPose(nullposition, constraints);
    }

    @Override
    public void execute() {
        pathfindcommand.execute();
        // Add execution code here
    }

    @Override
    public boolean isFinished() {
        return pathfindcommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathfindcommand.end(interrupted);
        // Add end code here
    }

}
