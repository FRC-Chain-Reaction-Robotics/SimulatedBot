package frc.robot.subsystems;

import java.io.File;
import java.io.FileReader;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    File swerveJsonDirectory;
    SwerveDrive swerveDrive;
    HolonomicPathFollowerConfig pathFollowerConfig;

    public SwerveSubsystem() {
        // For the JSON Configuration of Swerve, see the src/main/deploy/swerve
        // directory.
        // (This is currently unnessecary as its only required for a physical bot but
        // the basic structure is already there, values just need to be added to the
        // JSON structure)
        swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        JSONObject driveProperties, pidfDriveConfig, pidfAngleConfig;

        try {
            // Create Swerve Drive
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.MAX_SPEED);

            // Get nessecary configs for AutoBuilder configuration
            JSONParser parser = new JSONParser();
            JSONObject configFile = (JSONObject) parser
                    .parse(new FileReader(new File(swerveJsonDirectory, "modules/physicalproperties.json")));
            JSONObject pidfConfigFile = (JSONObject) parser
                    .parse(new FileReader(new File(swerveJsonDirectory, "modules/pidfproperties.json")));
            driveProperties = (JSONObject) ((JSONObject) configFile.get("conversionFactors")).get("drive");
            pidfDriveConfig = (JSONObject) pidfConfigFile.get("drive");
            pidfAngleConfig = (JSONObject) pidfConfigFile.get("angle");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants( // translation constants
                (Double) pidfDriveConfig.get("p"),
                (Double) pidfDriveConfig.get("i"),
                (Double) pidfDriveConfig.get("d"),
                (Double) pidfDriveConfig.get("iz")
            ),
            new PIDConstants( // rotation constants
                (Double) pidfAngleConfig.get("p"),
                (Double) pidfAngleConfig.get("i"),
                (Double) pidfAngleConfig.get("d"),
                (Double) pidfAngleConfig.get("iz")
            ),
            Constants.Swerve.MAX_SPEED,
            ((Double) driveProperties.get("diameter"))/2,
            new ReplanningConfig(true, true)
        );

        // These two final lines are only needed for simulation purposes, they are
        // configured based on control method of a real bot
        if (!Robot.isReal()) {
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
        }

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                        // ChassisSpeeds. Also optionally outputs individual module
                                                        // feedforwards
                pathFollowerConfig, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
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

    public DoubleSupplier getMeasurementSource() {
        return () -> {
            return swerveDrive.getPose().getX();
        };
    }

    public DoubleConsumer driveForward() {
        return (double speed) -> { 
            swerveDrive.driveFieldOriented(new ChassisSpeeds(speed, 0, 0.0)); 
        };

    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void driveRobotRelative(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        if (Robot.isReal()) {
            swerveDrive.resetOdometry(pose);
        }
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerveDrive.getRobotVelocity();
    }
}
