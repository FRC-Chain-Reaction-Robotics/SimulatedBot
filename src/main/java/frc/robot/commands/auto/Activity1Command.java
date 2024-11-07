package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class Activity1Command extends PIDCommand {

    PathPlannerPath path;
    static final double distance = 10.0;

    public Activity1Command(double p, double i, double d, SwerveSubsystem mSwerve) {
        super(new PIDController(p, i, d),
         mSwerve.getMeasurementSource(), 
         distance, 
         mSwerve.driveForward(), 
         mSwerve);

        getController().setTolerance(1.0);
        
    
    }

    public void initialize() {

    }

    public void end(boolean interrupted) {
    }
}
