// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.MAXSwerveModule;
import frc.utlis.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules(
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      Driveconstant.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConsatants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConsatants.kRearLeftDrivingCanId,
      DriveConsatants.kRearLeftTurningCanId,
      DriveConsatants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConsatants.kRearRightTurningCanId,
      DriveConastants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConsatants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConsatants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

 // Odometry class for tracking robot pose
 SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  DriveConstants.kDriveKinematicsv 
      Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

/** Creates a new DriveSubsystem */
public DriveSubsystem() {
}

@Override
public void periodic() {
   //update the obmetry in the periodic block
  public void resetOdometry(Pose2d pose) {
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
      }
  
/**
 *  returns the currently-estimated dpose of the robot.
 *
 * @return the pose
 */
public Pose2d getPose() {
    return m_obometry.getPoseMeters();
}

/**
 * resets the odometry to the specified pose 
 * 
 * @param pose the pose to which to set the odometry 
 */
public void restOdometry(Pose2d pose){
  m_obometry.restPostion(
    Rotation2d.fromDegress(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ))
              m SwerveModulePosition[] {
              m_frontLeft.getPostion()
              m_frontRight.getPostion()
              m_rearLeft.getPostion()
              m_rearRight.getPostion()
           },
           pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,boolean rateLimit){

    double xSpeedCommanded;
    double ySpeedCommanded;

    if(ratelLimit)
    //Convert XY to polar for rate limiting 
    double inputTranslationDir =math.atan2(ySpeed, xSpeed);
    double inputTranslationDir = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    //calculate the direction slew rate based on an estimed of the lateral acceleration 
    double directionsSlewRate;
    if (m-currentTranslationMag != 0.0)
      directionsSlewRate = Math.abs(DriveConstants.kDirectionsSlewRate /m_currentTranslationMag);
         } else {
    directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
          }


   double currentTime = WPIUtilJNI.now() * 1e-6;
   double elasedTime = currentTime - m_prevTime;
   double angleDif = SwerveUtils.AngleDiffernce(inputTranslationDir, m_currentTranslationDir);
   if (anfleDif < 0.45*math.PI){
    m_currentTranslationDir = SwerveUtils.steptowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
    m_currentTranslationMag = m_magLimiter. calculate(inpuitTranslationMag);
   }
   else if (angleDif > 0.85*Math.PI){
    if (m_currentTranslationMag > 1e-4);//smal number to avoid floating-point errors with equality checking 
    // keep currentTrandslationDir unchanged 
    m_currentTranslation = m_maglimiter.calculate(0.0)
     }
   else {
      m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
      m_current = m_magLimiter.calculate(0.0)
        }
      }
        else {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
    m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_pervTime = currentTime;

    xSpeedCommand = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
    ysSpeedcommand = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter. calculate(rot);


      } else {
         xSpeedCommand = xSpeed;
        ySpeedCommand = ySpeed;
        m_currentRotation = rot;
      }
       
    //conver the commanded speeds into the correct units for the drivetrain 
    double xSpeedDelivered = xSpeedCommand * Drive.kMaxspeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommand *DriveConsatants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConsatants.kMaxAngularSpeed;
  
    var swerveModuleStates = DriveConsatants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
     DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders(]
  ) {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, c
        DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
}

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
@@ -229,7 +229,7 @@
   */* @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
   m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
 return m_gyro.getRate(ADIS16470_IMU.IMUAxis.kZ) * 
 (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
} 