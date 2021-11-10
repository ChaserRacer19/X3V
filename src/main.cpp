#include "vex.h"
#include "MiniPID.h"
using namespace vex;
competition Competition;
//#define rev reverse
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int f1 = 0;
int f2 = 0;
int f3 = 0;
int f4 = 0;
int tog = 1;
int tog2 = 0;
int tog3 = 1;
int ringCheck = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
void turnPIDTime (double target, double time, bool reset = true, double i = 0) {
  MiniPID pid = MiniPID(0.27, i, 1.8);
  pid.setOutputLimits(-127, 127);
  pid.setMaxIOutput(30);
  pid.setSetpointRange(900);
  //int bias = std::nearbyint(0.059722*target);
  int bias = 0;
  //bias was 38
  float gyroCalibrationConst = 1;
  if(target < 0) {
    gyroCalibrationConst = 1.066;
  } else if (target > 0) {
    gyroCalibrationConst = 1.061;
  }
  //reset the gyro value
  if(reset){
    tom.resetHeading();
  }
  int iterations = 0;
  //int timeout = 0;
  while(iterations < time) {
     double output = pid.getOutput(tom.orientation(yaw,deg)/gyroCalibrationConst, target + bias);
     lMotor1.spin(fwd,-output,volt);
     lMotor2.spin(fwd,-output,volt);
     rMotor1.spin(fwd,output,volt);
     rMotor2.spin(fwd,output,volt);
     wait(40,msec);
     iterations = iterations + 10;
  }
  lMotor1.spin(fwd,0,volt);
  lMotor2.spin(fwd,0,volt);
  lMotor1.spin(fwd,0,volt);
  rMotor2.spin(fwd,0,volt);
}

//Helper Functions
void resetRotaions()
{
  lMotor1.resetPosition();
  lMotor2.resetPosition();
  rMotor1.resetPosition();
  rMotor2.resetPosition();
} //resetEncoders

void driveOn(int leftPower, int rightPower)
{
	lMotor1.spin(fwd,leftPower,volt);
  lMotor2.spin(fwd,leftPower,volt);
  rMotor1.spin(fwd,rightPower,volt);
  rMotor2.spin(fwd,rightPower,volt);
  
}
float getPercentSpeed( int count )
{
	float percent;

	if( count < 10 )
	{
		percent = count * 0.1;
	}
	else
	{
		percent = 1.0;
	}

	return ( percent );
}

int secondsToMsec( float seconds )
{
	return ( seconds * 1000 );
} //secondsToMsec


 //int t1 = timer();

//PID STARTS HERE 
void drive(int inches, float waitTimeSec, float speed)
{
  con.Screen.clearScreen();
  con.Screen.setCursor(1, 1);
  con.Screen.print(timer());
  con.Screen.print(rMotor1.position(deg));

  const int ticks = inches*(180/(4.1*3.14159));

  float T1=0;

  //const int MAX_SPEED = 127; //maximum speed as allowed by motors
  const int MIN_SPEED = 10; 

  float kP_dist = .4;
	float kI_dist = 0.0001;
	float kD_dist = 0.2;

	float kP_diff = .54;
	float kI_diff = 0.01;
	float kD_diff = .2;

  	int error_dist = 0;
	int error_diff = 0;

	int lastError_dist = 0;
	int lastError_diff = 0;

  	float integral_dist = 0;
	int derivative_dist;

	float integral_diff;
	int derivative_diff;

  int timeLimitMsec = secondsToMsec(waitTimeSec);

	int power_dist;
	int power_diff;

	float percentSpeed;
	int iteration = 1;

	float	tot_error_diff = 0;
	float averageError_diff = 0;

  //int rAverage =(rMotor1.position(deg)+rMotor2.position(deg)+rMotor3.position(deg))/3;
  //int lAverage =(lMotor1.position(deg)+lMotor2.position(deg)+lMotor3.position(deg))/3;
  resetRotaions();
  
  

  while(T1<timeLimitMsec && rMotor1.position(deg)<ticks)
  {
    
    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print(T1);
    con.Screen.setCursor(2, 2);
    con.Screen.print(rMotor1.position(deg));


  

    error_dist = ticks - ((lMotor1.position(deg)+rMotor1.position(deg))/2);
		error_diff = lMotor1.position(deg) - rMotor1.position(deg);

    //slowly ramp up speed over 40 loops
    percentSpeed = getPercentSpeed( iteration );
		iteration ++;

    	if( error_diff != 0)
		{
			integral_diff = integral_diff + error_diff;
		}
		else
		{
			integral_diff = 0;
		}
    derivative_dist = error_dist - lastError_dist;
		derivative_diff = error_diff - lastError_diff;

		lastError_dist = error_dist;
		lastError_diff = error_diff;

		power_dist = (error_dist * kP_dist) + (integral_dist * kI_dist) + (derivative_dist * kD_dist);
		power_diff = (error_diff * kP_diff) + (integral_diff * kI_diff) + (derivative_diff * kD_diff);

    if( abs(power_dist) > speed )
		{
			power_dist = speed * abs(power_dist)/power_dist;
		}

    int leftMotorSpeed =  MIN_SPEED + (power_dist - power_diff) * percentSpeed;
		int rightMotorSpeed = MIN_SPEED + (power_dist + power_diff) * percentSpeed;

    //driveOn( leftMotorSpeed, rightMotorSpeed );
   lMotor1.spin(fwd,leftMotorSpeed,volt);
   lMotor2.spin(fwd,leftMotorSpeed,volt );
   rMotor1.spin(fwd,rightMotorSpeed,volt);
   rMotor2.spin(fwd,rightMotorSpeed,volt );
   
   
    tot_error_diff = tot_error_diff + error_diff;
		averageError_diff = tot_error_diff / iteration;

    wait(40,msec);
    T1+=250;
    con.Screen.print(rMotor1.position(deg));
  }
    rMotor1.spin(fwd,0,volt);
    lMotor1.spin(fwd,0,volt);
    rMotor2.spin(fwd,0,volt);
    lMotor2.spin(fwd,0,volt);
    rMotor1.setBrake(brake);
    lMotor1.setBrake(brake);
    rMotor2.setBrake(brake);
    lMotor2.setBrake(brake);
    
    
    
  
}

//Arm PID
void arm(int amount, float timeWait, float fastness)
{
  const int ticksHight = amount*(180/(4.1*3.14159));

  float T1=0;

  //const int MAX_SPEED = 127; //maximum speed as allowed by motors
  const int MIN_SPEED = 10; 

  float kP_hight = 1.8;
	float kI_hight = 0.0001;
	float kD_hight = 0.2;

  int error_hight = 0;


	int lastError_hight = 0;
	
  float integral_hight = 0.0;
	int derivative_hight;

  int timeLimitMsec = secondsToMsec(timeWait);

	int power_hight;
	

	float percentSpeed;
	int iteration = 1;


  //int rAverage =(rMotor1.position(deg)+rMotor2.position(deg)+rMotor3.position(deg))/3;
  //int lAverage =(lMotor1.position(deg)+lMotor2.position(deg)+lMotor3.position(deg))/3;
  resetRotaions();
  
  

  while(T1<timeLimitMsec && ml2.position(deg)<ticksHight)
  {
    
    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print(T1);
    con.Screen.setCursor(2, 2);
    con.Screen.print(rMotor1.position(deg));


    //slowly ramp up speed over 40 loops
    percentSpeed = getPercentSpeed( iteration );
		iteration ++;

    derivative_hight = error_hight - lastError_hight;

		lastError_hight = error_hight;


		power_hight = (error_hight * kP_hight) + (integral_hight * kI_hight) + (derivative_hight * kD_hight);

    if( abs(power_hight) > fastness )
		{
			power_hight = fastness * abs(power_hight)/power_hight;
		}

    int leftMotorSpeed =  MIN_SPEED + (power_hight) * percentSpeed;
		int rightMotorSpeed = MIN_SPEED + (power_hight) * percentSpeed;

    //driveOn( leftMotorSpeed, rightMotorSpeed );
   ml2.spin(fwd,leftMotorSpeed,volt);
   ml3.spin(fwd,rightMotorSpeed,volt );
  
    wait(40,msec);
    T1+=250;
    con.Screen.print(rMotor1.position(deg));
  }
    ml2.spin(fwd,0,volt);
   ml3.spin(fwd,0,volt );
}

/*void drfwd(double dis,double speed){
  int a = 700;
  lMotor1.setVelocity(speed, pct);
  lMotor1.rotateFor(dis*(a/(12.88)),  rotationUnits::raw,false);
  lMotor2.setVelocity(speed, pct);
  lMotor2.rotateFor(dis*(a/(12.88)), rotationUnits::raw,false);
  rMotor1.setVelocity(speed, pct);
  rMotor1.rotateFor(dis*(a/(12.88)), rotationUnits::raw,false);
  rMotor2.setVelocity(speed, pct);
  rMotor2.rotateFor(dis*(a/(12.88)), rotationUnits::raw,false);
}

void drbwd(double dis,double speed){
  int a = 700;
  lMotor1.setVelocity(speed, pct);
  lMotor1.rotateFor(-1*dis*(a/(12.88)),  rotationUnits::raw,false);
  lMotor2.setVelocity(speed, pct);
  lMotor2.rotateFor(-1*dis*(a/(12.88)), rotationUnits::raw,false);
  rMotor1.setVelocity(speed, pct);
  rMotor1.rotateFor(-1*dis*(a/(12.88)), rotationUnits::raw,false);
  rMotor2.setVelocity(speed, pct);
  rMotor2.rotateFor(-1*dis*(a/(12.88)), rotationUnits::raw,false);
}
void drleft(double rot,double speed){
  int a = 700;
  double b = 7.93;
  lMotor1.setVelocity(speed, pct);
  lMotor1.rotateFor((-1)*2*3.14159*b*(rot/360)*(a/(12.88)),  rotationUnits::raw,false);
  lMotor2.setVelocity(speed, pct);
  lMotor2.rotateFor((-1)*2*3.14159*b*(rot/360)*(a/(12.88)), rotationUnits::raw,false);
  rMotor1.setVelocity(speed, pct);
  rMotor1.rotateFor((1)*2*3.14159*b*(rot/360)*(a/(12.88)), rotationUnits::raw,false);
  rMotor2.setVelocity(speed, pct);
  rMotor2.rotateFor((1)*2*3.14159*b*(rot/360)*(a/(12.88)), rotationUnits::raw,false);
}

void drright(double rot,double speed){
  int a = 700;
  double b = 7.93;
  lMotor1.setVelocity(speed, pct);
  lMotor1.rotateFor((1)*2*3.14159*b*(rot/360)*(a/(12.88)),  rotationUnits::raw,false);
  lMotor2.setVelocity(speed, pct);
  lMotor2.rotateFor((1)*2*3.14159*b*(rot/360)*(a/(12.88)), rotationUnits::raw,false);
  rMotor1.setVelocity(speed, pct);
  rMotor1.rotateFor((-1)*2*3.14159*b*(rot/360)*(a/(12.88)), rotationUnits::raw,false);
  rMotor2.setVelocity(speed, pct);
  rMotor2.rotateFor((-1)*2*3.14159*b*(rot/360)*(a/(12.88)), rotationUnits::raw,false);

}*/

void gOpen(){
  mG.open();
}

void gClose(){
  mG.close();
}


void bLift(int height, int speed){
ml3.setVelocity(speed, pct);
ml3.rotateFor(fwd,height*7,deg);
}

void ringOn(){
  rl.setVelocity(100,pct);
  rl.spin(fwd);
}

void ringOff(){
 rl.setVelocity(0,pct);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pre_auton(void) {
  vexcodeInit();
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void autonomous(void) {
   drive(36,5,100);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void usercontrol(void) {
  michia.resetPosition();
  while (1) {

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(lMotor1.position(deg));


    int left = con.Axis3.position()+con.Axis4.position();
    int right = con.Axis3.position()-con.Axis4.position();
    lMotor1.spin(fwd, left, percent);
    lMotor2.spin(fwd, left, percent);
    rMotor1.spin(fwd, right, percent);
    rMotor2.spin(fwd, right, percent);
  
con.Screen.clearScreen();
con.Screen.setCursor(1, 1);
con.Screen.print(michia.position(rev));


if (con.ButtonL1.pressing()) {
      if (f4 == 0) {
      f4 = 1;
      }
//}

//if (f4 == 1) {
  if (tog2 == 0) {
      ml1.spinTo(365*1.05, deg, false);
      tog2=1;
    /*
    if (michia.position(rev) <= 1.02) {
      ml1.spin(fwd, 50 ,pct);;
      con.Screen.print("tog1 spin fwd");
    } else if (michia.position(rev) >= 1.17) {
      ml1.spin(reverse, 50, pct);
      con.Screen.print("tog1 spin rev");
    } else {
      ml1.stop(hold);
      tog2 = 1;
      f4 = 0;
    }
    */
  } else if (tog2==1) {
      ml1.spinTo(365*.65, deg, false);
      tog2=0;

    /*
    if (michia.position(rev) <= .51) {
      ml1.spin(fwd, 10 ,pct);
      con.Screen.print("tog2 spin fwd");
    } else if (michia.position(rev) >= .62) {
      ml1.spin(reverse, 40, pct);
      con.Screen.print("tog2 spin rev");
    } else {
      ml1.stop(hold);
      tog2 = 0;
      f4 = 0;
    }
    */
  }

}
//}

//.63


    if (con.ButtonL2.pressing()) {
      
      if (f1==0) {
        if (tog3==0) {
          mG.open();
          tog3 = 1;
        } else {
          mG.close();
          tog3 = 0;
        }
        f1=1;
      }
    } else {
        f1=0;
    }
    

    if (con.ButtonB.pressing()) {
      rl.spin(reverse, 100, percent);
    } else if (con.ButtonX.pressing()) {
      if(f2==0) {
        if (tog==0) {
          tog = 1;
        } else {
          tog = 0;
        }
      f2=1;
      }
    } 
    if(!con.ButtonX.pressing()){
      f2=0;
    }


    if (tog==0 && !con.ButtonB.pressing()) {
      if (rl.current(pct)<=90) { 
      rl.spin(fwd,100,percent);
      } else {
      rl.spin(reverse,100,percent);
      }
    } else if (tog==1 && !con.ButtonB.pressing()) {
      rl.stop();
    } 
    
    if(con.ButtonR1.pressing()) {
      ml3.spin(fwd, 100, percent);
      ml2.spin(fwd, 100, percent);
    } else if (con.ButtonR2.pressing()) {
      ml3.spin(reverse, 100, percent);
      ml2.spin(reverse, 100, percent);
    } else {
      ml3.stop(brakeType::hold);
      ml2.stop(brakeType::hold);
    }
    wait(20, msec); 
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
