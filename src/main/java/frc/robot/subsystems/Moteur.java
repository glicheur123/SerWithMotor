/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoteurCommand;

/**
 * Add your docs here.
 */
public class Moteur extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private RobotMap m_map = new RobotMap();
  private Victor motor = m_map.sp;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoteurCommand());
  }

  public void right()
  {
    motor.set(1);
  }

  public void left()
  {
    motor.set(-1);
  }

  public void stop()
  {
    motor.set(0);
  }
}
