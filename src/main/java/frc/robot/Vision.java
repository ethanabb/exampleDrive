package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;


public class Vision extends SubsystemBase {
    
    public Vision() {
        
    }


    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    public double limelight_aim_proportional() {
    // returns 0 if no target is found
    if(LimelightHelpers.getLimelightNTTableEntry("limelight-front", "tv").getDouble(0) == 0) {
        return 0.0;
    }

    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0175;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;

    }
    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional() {
        // returns 0 if no target is found
        if(LimelightHelpers.getLimelightNTTableEntry("limelight-front", "tv").getDouble(0) == 0) {
            return 0.0;
        }

        double kP = .05;

        // desired area
        double desiredArea = 10.0;

        // error
        double error = desiredArea - LimelightHelpers.getTA("limelight-front");
        // proportional control
        double targetingForwardSpeed = error * kP;

        // scale by drivetrain speed
        targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;

        return targetingForwardSpeed;
    }
    
    

    @Override
    public void periodic() {
        if(LimelightHelpers.getLimelightNTTableEntry("limelight-front", "tv").getDouble(0) == 1) {
            LimelightHelpers.setLEDMode_ForceBlink("limelight-front");
            System.out.println("While");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight-front");
        }
       
        
    }
}
