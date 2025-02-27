package frc.robot.Subsystems;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase {

  //private final TalonFX armMotor;
  private final ThriftyNova climberMotor;

  /**
   * This subsytem that controls the arm.
   */
  public Climber() {

    // Set up the arm motor as a brushed motor
    climberMotor = new ThriftyNova(ClimberConstants.CLIMBER_MOTOR_ID);
      climberMotor.setInverted(true);
      climberMotor.setBrakeMode(true);
      climberMotor.setMaxCurrent(CurrentType.STATOR, ClimberConstants.CLIMBER_STATOR);
      climberMotor.setMaxCurrent(CurrentType.SUPPLY, ClimberConstants.CLIMBER_SUPPLY);
  }

  @Override
  public void periodic() {
  }

  /**
   * This is a method that makes the climber move at your desired speed
   * Positive values make it pull in to the robot and negative values would unravel the winch.
   * 
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runClimber(double speed) {
    climberMotor.setPercent(speed);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
  }
}
