package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private static double maximumSpeed = Units.feetToMeters(17.1);
  private SwerveDrive m_swerveDrive;
  private final Field2d field = new Field2d();


 public SwerveSubsystem() {
    //create swerve directory
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    //Put position on shuffleboard
    Shuffleboard.getTab("Drive").add("Field", field);
    //parse swerve directory; if wrong parameters are used, process will be output in the terminal
    try {
        m_swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        m_swerveDrive.resetOdometry(new Pose2d(7, 3, new Rotation2d(0)));
        m_swerveDrive.zeroGyro();
    } catch (IOException e) {
        File f = new File(swerveJsonDirectory, "swervedrive.json");
        e.printStackTrace();
        
        // Critical! Re-throw to avoid half-initialized objects
        throw new RuntimeException("Failed to load swerve drive config", e);
    }
}
public Command zeroGyro(){
  return run (() -> {
    m_swerveDrive.zeroGyro();
  });
}

   
      /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      m_swerveDrive.driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      m_swerveDrive.getOdometryHeading().getRadians(),
                                                                      m_swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommandF(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
     
      //for if robot wants to be slow or fast, you can change values if needed
      double driveMultiplier = 5.0;
      double turnMultiplier = 0.05; 
      
      Translation2d scaledInputs = new Translation2d(translationX.getAsDouble() * driveMultiplier,
      translationY.getAsDouble() * driveMultiplier);
      

          m_swerveDrive.drive(
            scaledInputs,
            angularRotationX.getAsDouble() * turnMultiplier * m_swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false
        );
    }); 

   //OPTIONAL DRIVE COMMAND
      //   m_swerveDrive.drive(new Translation2d(translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
    //                                       translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()),
    //                     angularRotationX.getAsDouble() * m_swerveDrive.getMaximumChassisAngularVelocity(),
    //                     true,
    //                     false);
    // });
  //}
};

  @Override
public void periodic() {
    Pose2d currentPose = m_swerveDrive.getPose(); // or odometry.getPoseMeters()
    field.setRobotPose(currentPose);
}

}
