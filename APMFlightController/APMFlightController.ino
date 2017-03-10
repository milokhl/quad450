  #include <AP_Common.h>
  #include <AP_Math.h>
  #include <AP_Param.h>
  #include <AP_Progmem.h>
  #include <AP_ADC.h>
  #include <AP_InertialSensor.h>
  #include <AP_HAL.h>
  #include <AP_HAL_AVR.h>
  #include <PID.h>
  #include <stdlib.h>
  #include <AP_HAL_AVR_SITL.h>
  #include <AP_HAL_Empty.h>
  #include <AP_GPS.h>
  #include <AP_Baro.h>
  #include <Filter.h>
  
  //defining the motor output indices
  #define MOTOR_FL 0
  #define MOTOR_FR 1
  #define MOTOR_BL 2
  #define MOTOR_BR 3
  
  //defining the LED indicator pins on the APM
  #define A_LED_PIN 27
  #define C_LED_PIN 25
  
  
  //defining the PID object indices (all PIDs are stored in an array)
  PID pids[8];
  #define PID_ROLL_RATE 0
  #define PID_ROLL_STAB 1
   
  #define PID_PITCH_RATE 2
  #define PID_PITCH_STAB 3
 
  #define PID_YAW_RATE 4
  #define PID_YAW_STAB 5
  
  //rewriting the map function from Arduino
  long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
  
  // Hardware abstraction layer
  const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
  
  //creating the Inertial Sensor
  AP_InertialSensor_MPU6000 ins;
  
  //function that flashes the indicator LEDs on the APM
  static void flash_leds(bool on) {
    hal.gpio->write(A_LED_PIN, on);
    hal.gpio->write(C_LED_PIN, on);
  }
  
  //creating the barometer and a timer that is used to calculate vertical velocity
  AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
  static uint32_t timer;
  static uint32_t timer2;
  
  //defining GPS related stuff
  GPS         *gps;
  AP_GPS_Auto GPS(&gps);
  #define T6 1000000
  #define T7 10000000  
  
  
  //wrap function for yaw
  #define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
  #define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
  
  
  /*    SETUP    */
    
  void setup() {
    
    //SETUP THE UARTs
    hal.uartA->begin(57600); // Console
    hal.uartA->println("[SETUP] Initialized console.");
    hal.uartB->begin(38400); // GPS #1
    hal.uartC->begin(57600); // telemetry 1
    hal.uartA->println("[SETUP] Initialized uartB and uartC!");
    
    //setup the ESC outputs to operate at 490Hz (minimize averaging filter)
    hal.rcout->set_freq(0, 490);
    hal.rcout->set_freq(1, 490);
    hal.rcout->set_freq(2, 490);
    hal.rcout->set_freq(3, 490);
    hal.uartA->println("[SETUP] Set ESC output frequencies!");
    
    //set the servo PWM pins to operate at 50Hz
    hal.rcout->set_freq(5, 50);
    hal.rcout->set_freq(6, 50);
    
    //idk what this does?
    hal.rcout->enable_mask(0xFF);
    
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
    
    //starting up the barometer
    hal.gpio->pinMode(63, GPIO_OUTPUT);
    hal.gpio->write(63, 1);
    baro.init();
    baro.calibrate();
    
    // Initialise MPU6050 sensor
    ins.init(AP_InertialSensor::COLD_START,  
    		 AP_InertialSensor::RATE_100HZ,
    		 NULL);
    
    hal.gpio->pinMode(A_LED_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(C_LED_PIN, GPIO_OUTPUT);
    
    // Initialise MPU6050's internal sensor fusion (aka DigitalMotionProcessing)
    hal.scheduler->suspend_timer_procs();  // stop bus collisions
    ins.dmp_init();                        
    ins.push_gyro_offsets_to_dmp();
    hal.scheduler->resume_timer_procs();
    
    //auto-level the accelerometer and gyro
    ins.init_accel(flash_leds);
    
    //Set up the PID gains
    pids[PID_PITCH_RATE].kP(0.65);
    pids[PID_PITCH_RATE].kI(1.0);
    pids[PID_PITCH_RATE].imax(50);
    
    pids[PID_ROLL_RATE].kP(0.55);
    pids[PID_ROLL_RATE].kI(0.9);
    pids[PID_ROLL_RATE].imax(50);
    
    pids[PID_YAW_RATE].kP(2.7);
    pids[PID_YAW_RATE].kI(1.0);
    pids[PID_YAW_RATE].imax(50);
    
    pids[PID_PITCH_STAB].kP(3.5);
    pids[PID_ROLL_STAB].kP(3.5);
    pids[PID_YAW_STAB].kP(8);
     
    //GPS setup
    gps = &GPS;
    gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
    
    //take the first timer reading (this is used for the barometer)
    timer = hal.scheduler->micros();
    timer2 = hal.scheduler->micros();
    
    hal.uartA->println("[SETUP] Initialized GPS!");
    hal.uartA->println("[SETUP] COMPLETE! Ready for takeoff.");  
  }
    
    /*    LOOP    */
    
    float yaw_target = 0;
    float roll, pitch, yaw;
    int counter = 0;
    int alt_lock=0; //0 or 1, depending on whether altitude lock is OFF/ON
    int camera_manual_mode=0; // also 0 or 1 for OFF/ON
    float baro_alt=0; //altitude reported by the barometer (meters)
    float baro_vv=0; //the vertical velocity reported by the baro (m/s)
    float desired_vv = 0; //the desired vertical velocity, determined by PID_ALT_RATE
    float desired_alt = 0; // desired altitude (meters)
    long alt_output = 0; // an output term that is sent to the motors when altitude lock is engaged
    long thr_at_alt_lock = 1500; //the quadcopter stores the throttle command that it received right before entering an altitude hold
                                  //this throttle is the starting point for the quadcopter to make altitude adjustments
                                  
    //Inertial sensor variables
    Vector3f accel, gyro;
    float accelX, accelY, accelZ;
    float zVel = 0;
    
    //GPS variables
    float lat, lon, gps_alt, ground_speed;
    int ground_course, num_sats, gps_time, gps_status;
    
    long ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7;
    long rcthr, rcyaw, rcpit, rcroll, camera_pitch, camera_roll;
    
    float gyroPitch, gyroRoll, gyroYaw;
    
    void loop() {
      counter++; //counter is used to control the rate of print statements
    
      //makes the loop wait until new accelerometer data comes in
      //while (ins.num_samples_available() == 0);
    
      //read in the RC values
      ch0 = hal.rcin->read(0);
      ch1 = hal.rcin->read(1);
      ch2 = hal.rcin->read(2);
      ch3 = hal.rcin->read(3);
      ch4 = hal.rcin->read(4);
      ch5 = hal.rcin->read(5);
      ch6 = hal.rcin->read(6);
      ch7 = hal.rcin->read(7);
    
      //map the RC values to a usable range
      rcthr = map(ch2, 1067, 1900, 1000, 2000);
      rcyaw = map(ch3, 1060, 1903, -150, 150);
      rcpit = map(ch1, 1062, 1901, -45, 45);
      rcroll = -1 * map(ch0, 1058, 1898, -45, 45);
      camera_roll = map(ch5, 1061, 1901, 1010, 1990);
      camera_pitch = map(ch6, 1074, 1901, 1010, 1990);
      
      //get pitch, roll, yaw
      ins.update(); //moved this line up to the manual control mode
      ins.quaternion.to_euler(&roll, &pitch, &yaw);
      roll = ToDeg(roll)-3.9;
      pitch = ToDeg(pitch)+5.3;
      yaw = ToDeg(yaw);

      //get angular velocities
      gyro = ins.get_gyro();
      gyroPitch = ToDeg(gyro.y);
      gyroRoll = ToDeg(gyro.x);
      gyroYaw = ToDeg(gyro.z);
      
      //get accelerometer data
      accel = ins.get_accel();
      accelX = accel.x, accelY = accel.y, accelZ = accel.z;
      timer = hal.scheduler->micros();
      
      //update the GPS and store the latest variables
      gps->update();
      if (gps->new_data) {
          if (gps->fix) {
            
              lat = gps->latitude;
              lon = gps->longitude;
              gps_alt = (float)gps->altitude / 100.0;
              ground_speed = (float)gps->ground_speed / 100.0;
              num_sats = gps->num_sats;
              ground_course = gps->ground_course / 100;
              gps_time = gps->time;
              gps_status = gps->status();
              
          } else {
              hal.uartA->println("No GPS fix.");
          }
          gps->new_data = false;
      }
      
      
      //put any debugging print statements inside of the if statement below
      /*      DEBUGGING     */ 
      if (counter > 20) {
          counter = 0;
          //hal.uartA->println(lat);
          //hal.uartA->print(desired_vv);
          //hal.uartA->print(" | ");
          //hal.uartA->print(alt_output);
          //hal.uartA->println();
          //hal.uartA->print(" | ");
          //hal.uartA->print(baro_alt);
          //hal.uartA->println();
          //hal.uartA->print(accelX);
          //hal.uartA->print(" | ");
          //hal.uartA->print(accelZ);
          //hal.uartA->print(" | ");
          //hal.uartA->print(zVel);
          //hal.uartA->print("\n");
          //hal.uartA->print(baro_alt);
          //hal.uartA->println();
      
      /*
          hal.uartA->printf_P(
              PSTR("P:%4.1f  R:%4.1f Y:%4.1f\n"),
        	  pitch,
        	  roll,
        	  yaw); */
        
       //print out the roll
        hal.uartA->printf_P(
              PSTR("D: %4.1f \n"),
        	  roll);
        /*
        hal.console->print("Lat: ");
              print_latlon(hal.console,gps->latitude);
              hal.console->print(" Lon: ");
              print_latlon(hal.console,gps->longitude);
              hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu STATUS: %u\n",
                            alt,
                            ground_speed,
                            ground_course,
                            num_sats,
                            gps_time,
                            gps->status()); */
      }
      /*      END DEBUGGING AREA      */
   
      
      //do not allow the throttle to exceed 80% so that the quad can still perform stabilization at full throttle
      if (rcthr > 1800) {
        rcthr = 1800;
      }
      
      if (rcthr > 1170) {
        
        //get the stabilization outputs
        float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
        float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
        float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
        
        //if pilot commands a change in yaw, set this to the yaw target
        if (abs(rcyaw) > 5) {
          yaw_stab_output = rcyaw;
          yaw_target = yaw;
        }
        
        //feed the stabilization outputs into the rate PIDs
        long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -250, 250);  
        long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -250, 250);  
        long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -250, 250);  
          
        //write outputs to the motors
        hal.rcout->write(MOTOR_FL, rcthr - roll_output - pitch_output + yaw_output);
  	hal.rcout->write(MOTOR_BL, rcthr - roll_output + pitch_output - yaw_output);
  	hal.rcout->write(MOTOR_FR, rcthr + roll_output - pitch_output - yaw_output);
  	hal.rcout->write(MOTOR_BR, rcthr + roll_output + pitch_output + yaw_output);          
        
    } else {  // MOTORS OFF
	hal.rcout->write(MOTOR_FL, 1000);
	hal.rcout->write(MOTOR_BL, 1000);
	hal.rcout->write(MOTOR_FR, 1000);
	hal.rcout->write(MOTOR_BR, 1000);

        //reset the yaw target when there is not throttle (yaw resets on ground)
        yaw_target = yaw; 
    }
      
 
  }
  
  // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).
  AP_HAL_MAIN(); 
    
