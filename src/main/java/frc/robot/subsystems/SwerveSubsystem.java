package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;

public class SwerveSubsystem extends SubsystemBase {
    File swerveJsonDirectory;
    SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        // For the JSON Configuration of Swerve, see the src/main/deploy/swerve
        // directory.
        // (This is currently unnessecary as its only required for a physical bot but
        // the basic structure is already there, values just need to be added to the
        // JSON structure)
        swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // These two final lines are only needed for simulation purposes, they are
        // configured based on control method of a real bot
        if (!Robot.isReal()) {
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
        }
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    translationX.getAsDouble(), 
                    translationY.getAsDouble(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumAngularVelocity()));
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }
}
