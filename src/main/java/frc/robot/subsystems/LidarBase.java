/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LidarBase extends SubsystemBase {

  private I2C i2c;
	private static byte[] distance;
	private java.util.Timer updater;
	private LIDARUpdater task;
	
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
  private final int LIDAR_DISTANCE_REGISTER = 0x8f;
  
  public LidarBase() {
    i2c = new I2C(Port.kMXP, LIDAR_ADDR);
		distance = new byte[2];
		task = new LIDARUpdater();
		updater = new java.util.Timer();
  }
	
	// Distance in inches
	public double getDistance() {
		return ((double)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]))* 0.393701;
  }
 
	public double pidGet() {
		return getDistance(); //add PID element
	}
	
	// Start 10Hz polling
	public void start() {
		updater.scheduleAtFixedRate(task, 0, 100);
	}
	
	// Start polling for period in milliseconds
	public void start(int period) {
		updater.scheduleAtFixedRate(task, 0, period);
	}
	
	public void stop() {
		updater.cancel();
	}
	
	// Update distance variable
	public void update() {
		i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement
		Timer.delay(0.04); // Delay for measurement to be taken
		i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance); // Read in measurement
		Timer.delay(0.01); // Delay to prevent over polling
	}
	
	// Timer task to keep distance updated
	private class LIDARUpdater extends TimerTask {
		public void run() {
			while(true) {
				update();
				SmartDashboard.putNumber("LIDAR distance Inches", getDistance());
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
