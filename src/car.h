
#ifndef CAR_H
#define CAR_H
class BehaviourPlanner;
class TrajectoryPlanner;

class car
{
public:
   double _x = 0;
   double _y = 0;
   double _s = 0;
   double _d = 0;
   double _yaw = 0;
   int _lane = 0;
   double _speed = 0;
   double _distance = 0;
   double _endS = 0;
   double _endD = 0;

   BehaviourPlanner* _bp;
   TrajectoryPlanner* _tp;


   car(BehaviourPlanner* bp = nullptr, TrajectoryPlanner* tp = nullptr) : _bp(bp), _tp(tp) {};
   
   void setValues(double x, double y, double s, double d, double yaw, int lane, double speed, double endS = 0, double endD = 0)
   {
	   _x = (x);
	   _y = (y);
	   _s = (s);
	   _d = (d);
	   _yaw = (yaw);
	   _lane = (lane);
	   _speed = (speed);
	   _endS = (endS);
	   _endD = (endD);

   };
 
};
#endif
