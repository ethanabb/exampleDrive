package frc.robot.subsystems.Drive;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
  private static SwerveDrive m_swerveDrive;
  //START OF POSE-ESTIMATOR Objects    
  private final SwerveDriveKinematics m_Kinematics = m_swerveDrive.kinematics;

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final Pose2d m_initPose2d = new Pose2d();

  private final SwerveDrivePoseEstimator m_poseEstimator; 

  private final Field2d m_field = new Field2d();

  private final ShuffleboardTab m_swerveTab = Shuffleboard.getTab("DriveSubsystem");

  //END OF POSE-ESTIMATOR Objects
  
  
  // Constructor
  public SwerveSubsystem() {
    // create swerve directory
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    //parse swerve directory; if wrong parameters are used, process will be output in the terminal
    try {
      m_swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
      m_swerveDrive.useExternalFeedbackSensor();
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } catch (IOException e) {
      File f = new File(swerveJsonDirectory, "swervedrive.json");
      e.printStackTrace();
      
      // Critical! Re-throw to avoid half-initialized objects
      throw new RuntimeException("Failed to load swerve drive config", e);
    }

    m_poseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, m_gyro.getRotation2d(), getModulePositions(), m_initPose2d);
    m_swerveTab.add("Field", m_field)
      .withWidget("Field2d"); // Explicitly set the widget type


  }
  
  // Reset Pose2d Odeometry
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }
  
  // Zero the acutal physical gyro on the bot
  public Command zeroGyro(){
    return run (() -> {
    m_swerveDrive.zeroGyro();
    });
  }

  // Gets the positions by getting distance and rotation from the encoders.
  // Yagsl does this for you, this method is how you make it do that.
  public SwerveModulePosition[] getModulePositions() {
    SwerveModule[] modules = m_swerveDrive.getModules();
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

    for (int i = 0; i < modules.length; i++) {
        // This method correctly returns a SwerveModulePosition object
        positions[i] = modules[i].getPosition();
    }

    return positions;
  }
  
  // Gets the pose of our Pose2d object
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  

  // We love driving. It's pretty important.
  public Command driveCommandF(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(
        SwerveMath.scaleTranslation(
          new Translation2d(
            translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(), 
            translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()), 
          0.8), 
          Math.pow(angularRotationX.getAsDouble(), 3) * m_swerveDrive.getMaximumChassisAngularVelocity(), 
          true, 
          false);
    });
  }

  public Command driveCommandL(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * m_swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
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


  @Override
  public void periodic() {
    // Updates Pose Estimator
    m_poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());

    // Update Pose on Field
    m_field.setRobotPose(getPose());

    // Limelight Stuff
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }



}
  




