// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private VictorSPX RightFront = new VictorSPX(0);
  private VictorSPX RightRear = new VictorSPX(1);
  private VictorSPX LeftFront = new VictorSPX(2);
  private VictorSPX LeftRear = new VictorSPX(3);

  private Joystick joy1 = new Joystick(0);

  public static final int leftMotorID = 15;
  public static final int rightMotorID = 14;

  CANSparkMax leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    // DriveTrain Control
    double speed = -joy1.getRawAxis(5) * 0.3;
    double turn = joy1.getRawAxis(0) * 0.3;

    // ClimbingArm Control
    double Pull = joy1.getRawAxis(3) * 0.3;
    double Push = joy1.getRawAxis(2) * 0.3;

    // DriveTrain Varibles
    double left = speed + turn;
    double right = speed - turn;

    // ClimbingArm Varible
    double climbSpeed = Pull - Push;

    /* Alternative Way
     * rightMotor.follow(leftMotor);
    */

    // DriveTrain
    RightFront.set(ControlMode.PercentOutput, 0 + right);
    RightRear.set(ControlMode.PercentOutput, 0 + right);
    LeftFront.set(ControlMode.PercentOutput, 0 + left);
    LeftRear.set(ControlMode.PercentOutput, 0 + left);

    // ClimbingArm
    rightMotor.set(0 + climbSpeed);
    leftMotor.set(0 + climbSpeed);

    // when ClimbingArm is extending, Pull = 1
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
