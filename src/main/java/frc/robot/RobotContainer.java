// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.Activity1Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem drivebase = new SwerveSubsystem();
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.15)
    );

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    // or alternatively to select a default use
    // autoChooser = AutoBuilder.buildAutoChooser("Default Auto");
    
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {
      
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
