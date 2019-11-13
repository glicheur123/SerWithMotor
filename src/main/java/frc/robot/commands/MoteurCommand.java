/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Moteur;

public class MoteurCommand extends Command {

  private SerialPort ser;

  public MoteurCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_Moteur);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    ser = new SerialPort(9600, SerialPort.Port.kUSB);
    
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_Moteur.right();
    String texte = "0";
    if(ser.readString(2).isEmpty())
    {
      texte = "0";
    }else{
      texte = ser.readString(2);
    }
    //String texte = serial.readString();
    int test = Integer.parseInt(texte.trim());
    System.out.println("\"" + test + "\"");
    if(test == 20)
    {
      Robot.m_Moteur.stop();
      try {
        Thread.sleep(2000);
      } catch (Exception e) {
        //TODO: handle exception
      }
      texte = "0";
    }else{
      Robot.m_Moteur.right();
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_Moteur.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
