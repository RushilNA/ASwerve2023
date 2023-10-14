// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

  import edu.wpi.first.math.geometry.Translation2d;
  import edu.wpi.first.math.kinematics.ChassisSpeeds;
  import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
  import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
  import edu.wpi.first.math.kinematics.SwerveModulePosition;
  import edu.wpi.first.wpilibj.AnalogGyro;
  import com.kauailabs.navx.frc.AHRS;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  
  public static final double kMaxSpeed = 3; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(2, 1, 2);
  private final SwerveModule m_frontRight = new SwerveModule(6, 5, 4);
  private final SwerveModule m_backLeft = new SwerveModule(4, 3, 1);
  private final SwerveModule m_backRight = new SwerveModule(8, 10, 3);

  private final AHRS m_gyro = new AHRS();

  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }
  public void dashboardValues() {
    // You can add other values as needed
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getRotation2d().getDegrees());
}


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModule getFrontLeft() {
        return m_frontLeft;
    }

  public SwerveModule getFrontRight() {
        return m_frontRight;
    }

  public SwerveModule getBackLeft() {
        return m_backLeft;
    }

  public SwerveModule getBackRight() {
        return m_backRight;
    }


  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
