// ---------- stepper.h ----------
// #include <SSLClient.h> in wwe.ino BREAKS the min() and max() functions during compile, but only in this file!
// see https://forum.arduino.cc/t/max-and-min-functions-not-available/319076
// So, we explicitly redefine them here:
inline int max(int a,int b) {return ((a)>(b)?(a):(b)); }
inline int min(int a,int b) {return ((a)<(b)?(a):(b)); }
// Amazingly, this fixes the problem.

//
#define ACCELERATION_INIT 5     // 5 usteps/s per state machine evaluation * 1000 evals/s = 5000 usteps/s/s
#define MIN_VELOCITY_INIT 172   // 0.5 deg/s * 2000 usteps/360 deg * 62 = 172 usteps/s
#define MAX_VELOCITY_INIT 6889  // 20 deg/s * 2000 usteps/360 deg * 62 rev/rev = 6889 usteps/s

#define CHECK_BIT(var,pos) !!((var) & (1<<(pos)))
#define MOTOR_ENABLED LOW
#define MOTOR_DISABLED HIGH
#define MOTOR_DIR_RIGHT HIGH
#define MOTOR_DIR_LEFT LOW
#define STEPPERCTL_PER_SEC 1000  // 1000 is max allowable - see readADCs() in adc.ino

class StepperMotor{
  public:
    // constructor
    StepperMotor(Tc* motor_timer_id, int motor_timer_num, int step_pin, int dir_pin, 
            int enbl_pin, int acceleration, int min_vel, int max_vel): 
            motor_timer_id(motor_timer_id), motor_timer_num(motor_timer_num), 
            step_pin(step_pin), dir_pin(dir_pin), enbl_pin(enbl_pin), 
            acceleration(acceleration), min_velocity(min_vel), 
            max_velocity(max_vel), current_position(0), desired_position(0), 
            current_velocity(min_vel), motor_dir(LOW), motor_enbl(MOTOR_DISABLED){
    }
    void Acceleration(int a){
      acceleration = a;
    }
    int Acceleration(){
      return(acceleration);
    }
    void maxVelocity(int v){
      max_velocity = v;
    }
    int maxVelocity(){
      return(max_velocity);
    }
    void currentVelocity(int v){
      current_velocity  = v;
    }
    int currentVelocity(){
      return(current_velocity);
    }
    void currentPosition(int x){
      current_position = x;
    }
    int currentPosition(){
      return(current_position);
    }
    void desiredPosition(int x){
      desired_position = x;
    }
    int desiredPosition(){
      return(desired_position);
    }

    int isMotorOn(){
      return(motor_enbl == MOTOR_ENABLED);
    }
    
    // return 1 for right, 0 for left
    int motorDir(){
      if(motor_dir == MOTOR_DIR_RIGHT){ 
        return(1);
      }else{
        return(0);
      }
    }

    int decelX(){
      return(decel_x);
    }

    void updateDecelX(){
      // If we are going at a particular velocity, seeking a particular position, we need to know 
      // at what displacement we need to start decelerating. This calculates how many steps it should
      // take, at the specified acceleration, it will take to decelerate to zero velocity.
      // 
      // v = v0 + at
      // With constant deceleration, time to decelerate to zero
      //  0 = v0 - at   so   t = v0 / a
      // distance traveled at constant deceleration: x = v0*t - at^2/2
      //    x = v0 * (v0 / a) - a * (v0^2 / a^2) / 2 
      //    x = (v0^2 / a) - (v0^2 / 2a) = v0^2 / 2a
      // Because acceleration = usteps/sec per state machine period, 
      // there is a factor of STATE_MACHINE_RATE in the denominator.
      // Calculating this way should be OK, because max value of current_velocity = 20deg/s = 6888 usteps/s
      // 6888^2 = 47,444,544 < (2^31 - 1) = 2,147,483,647
      //decel_x = (current_velocity/acceleration) * current_velocity / STEPPERCTL_PER_SEC / 4;  // cw's original calculation
      decel_x = (current_velocity * current_velocity) / (2 * acceleration) / STEPPERCTL_PER_SEC;  // aw revision
    }
    
    // Following will be called at regular intervals, timer driven:
    void updateState(){
      // Left of where we want to be
      if(current_position < desired_position){
        if(motor_enbl == MOTOR_DISABLED){
          motor_dir = MOTOR_DIR_RIGHT;
          motor_enbl = MOTOR_ENABLED;
        }
        // We're to the left of where we want to be.
        if(motor_dir == MOTOR_DIR_RIGHT && (desired_position - current_position) > decel_x){
#ifdef SHOW_MOTOR_STATUS
          writeStatus2(1);
#endif
          // Headed the correct direction. 
          // We are to the left of the point of needing to slow down, 
          // so accelerate, but not beyond max velocity.
          current_velocity = min( max_velocity, (current_velocity + acceleration) );
        }else{
          // (MOTOR_DIR == MOTOR_DIR_LEFT || desired_position - current_position <= decel_x)
          // We are either headed the wrong direction, so need to decelerate, or headed the right 
          // direction but it's time to decelerate.
          current_velocity = max( min_velocity, (current_velocity - acceleration) );
#ifdef SHOW_MOTOR_STATUS
          writeStatus2(3);
#endif          
        }
      // right of desired position  
      }else if(current_position > desired_position){
        if(motor_enbl == MOTOR_DISABLED){
          motor_dir = MOTOR_DIR_LEFT;
          motor_enbl = MOTOR_ENABLED;
        }
        // Right of where we want to be.
        if((motor_dir == MOTOR_DIR_LEFT)&&(current_position - desired_position > decel_x)){
          // Moving left and to the right of the point of needing to slow down, 
          // so accelerate, but not beyond max velocity.
#ifdef SHOW_MOTOR_STATUS
          writeStatus2(5);
#endif          
          current_velocity = min(max_velocity, current_velocity + acceleration);
        } else {
#ifdef SHOW_MOTOR_STATUS
          writeStatus2(7);
#endif          
          // (MOTOR_DIR == MOTOR_DIR_RIGHT || desired_position - current_position <= decel_x)
          // Headed the wrong direction, need to slow down and get turned around
          current_velocity = max(min_velocity, current_velocity - acceleration);
        }
      }else if(motor_enbl == MOTOR_ENABLED){
        // Exactly where we want to be but not stopped! Just passing through, need to decelerate.
        current_velocity = max(min_velocity, current_velocity - acceleration);
      } else {
#ifdef SHOW_MOTOR_STATUS
        writeStatus2(0);
#endif
        // Otherwise, we are stopped and where we want to be. No need to do anything.        
      }
      // Set Register C to the value for current_velocity.
      // This runs the timer/counter at twice the required rate, and the timer interrupt 
      // routine toggles the "step" output.
      current_rc = 42000000 / min(current_velocity, MAX_VELOCITY_INIT);
    }

    // Following is called from the motor timer interrupt. It generates the actual "step" signal that is sent
    // to the motor controller.
    
    void handleMotorInterrupt(){
      // This is for diagnostic purposes.
      digitalWriteDirect(STEPPER_MOTOR_INT_PIN, HIGH);
      static int lastRC = 0;
      // Make leading edge of motor clock.
      if(outstate == HIGH){
        if(motor_enbl == MOTOR_ENABLED){
          digitalWriteDirect(MOTOR_STEP_PIN, HIGH);
          // leading edge of step pulse
          // If We have a new value for RC, then update the register value.
          //if(lastRC != current_rc){
            // The actual timer runs at twice the rate of our stepper clock, so we divide RC by 2 and RA by 4 to make that happen.
            TC_SetRC(motor_timer_id, motor_timer_num, current_rc >> 1);
            TC_SetRA(motor_timer_id, motor_timer_num, current_rc >> 2);
            lastRC = current_rc;
          //}
          // Keep track of current position.
          if(motor_dir == MOTOR_DIR_RIGHT){
            current_position++;
          }else{
            current_position--;
          }
          // If we've made it to where we're goinga and going slowly enough, stop immediately. 
          // Next time through here, the output will be suppressed.
          //if((current_position == desired_position) && (current_velocity < (3 * min_velocity))){
          if((current_velocity == min_velocity)
                || ((current_position == desired_position) && (current_velocity < (10 * min_velocity)))){  
            motor_enbl = MOTOR_DISABLED;
            current_velocity = min_velocity;
          }
        }
      }else{
        // Make trailing edge of clock
        digitalWriteDirect(MOTOR_STEP_PIN, LOW);
        // Set direction pin according to motor_dir.
        digitalWriteDirect(MOTOR_DIR_PIN, motor_dir);
        digitalWriteDirect(MOTOR_ENBL_PIN, motor_enbl);
      }
      // keep track of the current waveform state.
      outstate = ~outstate  & 1;
      digitalWriteDirect(STEPPER_MOTOR_INT_PIN, LOW);
    }
  protected:
    int decel_x = 0;
    int cycle_num = 0;
    // Hardware timer id
    Tc* motor_timer_id;
    int motor_timer_num;
    int32_t acceleration; // in steps / second^2
    int32_t max_velocity; // in steps/second
    int32_t min_velocity; // in steps per second
    int32_t desired_position; // 0 means "neutral", can go both positive and negative.
    int32_t current_position; // This, too.
    int32_t current_velocity; // In steps per second.
    int32_t current_rc;
    // motor direction
    int motor_dir = MOTOR_DIR_RIGHT;
    int motor_enbl = MOTOR_DISABLED;
    // Pin numbers motor control signals"
    int step_pin;
    int dir_pin;
    int enbl_pin;
    int outstate;
};



// THIS CLASS HAS NOT BEEN USED...
//   in favor of using functions in furlctl.ino, specifically in furlctl1().
//   Instantiate as such: TailPositioner tail_positioner(motor, &debounced_rs_state);

// TailPositioner manages a StepperMotor that moves a tail through a slewing drive.
//   The motor has 200 steps per revolution and the slewing drive multiplies that
//   by 62, so there are 12400 motor steps per full revolution of the slewing drive.
//
// We drive the stepper motor with an Anaheim Automation MBC12101 driver, which
//   does 10 microsteps per motor step. So, the position measurements in microsteps 
//   that we use are 10x what the motor sees. That is, one revolution of the slewing 
//   drive takes 124000 microsteps.
// 
// We allow +/- 90 degrees of movement of the tail, so 62000 microsteps total. The 
//   furthest extent of travel should be +/- 31000 microsteps.

class TailPositioner{
  public:
    TailPositioner(StepperMotor themotor, boolean* prs): motor(themotor), ptr_debounced_rs_state(prs){}

    // kick off the initialization process.
    void orient(boolean direction){
      // start going to the right
      if(direction == 1){
        motor.currentPosition(0);
        motor.desiredPosition(62000);
        seek_right = true;
      }else{
        motor.currentPosition(0);
        motor.desiredPosition(-62000);
        seek_left = true;
      }
    }

    // should be called at a regular interval to control motor behavior 
    // if needed.
    void update(){
      if(seek_right){
        // Step right until we see the reed switch close
        if(*ptr_debounced_rs_state == LOW){
          motor.currentPosition(31000);
          motor.desiredPosition(0);
          seek_right = false;
        }
      }
      if(seek_left){
        // Step left until we see the reed switch close
        if(*ptr_debounced_rs_state == LOW){
          motor.currentPosition(-31000);
          motor.desiredPosition(0);
          seek_left = false;
        }
      }
    }
    boolean seekingRight(){
      return(seek_right);
    }
    boolean seekingLeft(){
      return(seek_left);
    }
  private:
    boolean seek_right = false;
    boolean seek_left = false;
    StepperMotor motor;
    boolean* ptr_debounced_rs_state;
};
