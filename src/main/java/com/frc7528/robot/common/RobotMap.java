package com.frc7528.robot.common;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



public class RobotMap {
    //Drivetrain
    public static WPI_TalonFX m_leftAft = new WPI_TalonFX(1);
    public static WPI_TalonFX m_leftFront = new WPI_TalonFX(2);
    public static WPI_TalonFX m_rightAft = new WPI_TalonFX(3);
    public static WPI_TalonFX m_rightFront = new WPI_TalonFX(4);
    public static DifferentialDrive m_drive;

    public static WPI_TalonSRX controlPanelWheel = new WPI_TalonSRX(0);


    //Operator interface
    public static Joystick m_joy = new Joystick(0);
    // Conveyeor Belt
    public static TalonSRX ConveyorMotor1 = new TalonSRX(7);
    public static TalonSRX ConveyorMotor2 = new TalonSRX(15);
    // Turret
    public static TalonSRX Turret = new TalonSRX(51); // This shoudl be turret 20
    // Flywheel
    public static TalonSRX FlyL_Aft = new TalonSRX(20); //This should be FlyL_Aft 51
    public static TalonSRX FlyR_Front = new TalonSRX(50);
    
}
