#include "map.h"

Map::Map(const std::string& map_file)
{
  std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    maps_x.push_back(x);
    maps_y.push_back(y);
    maps_s.push_back(s);
    maps_dx.push_back(d_x);
    maps_dy.push_back(d_y);
  }

  // Add the last point to the beginning of the list
  maps_s.insert(maps_s.begin(), maps_s.back() - MAX_S);
  maps_x.insert(maps_x.begin(), maps_x.back());
  maps_y.insert(maps_y.begin(), maps_y.back());
  maps_dx.insert(maps_dx.begin(), maps_dx.back());
  maps_dy.insert(maps_dy.begin(), maps_dy.back());

  // Add the originally beginning two points (now index 1-2) to the end of the list
  for (size_t i = 1; i <= 2; i++) {
    maps_s.push_back(MAX_S + maps_s[i]);
    maps_x.push_back(maps_x[i]);
    maps_y.push_back(maps_y[i]);
    maps_dx.push_back(maps_dx[i]);
    maps_dy.push_back(maps_dy[i]);
  }

  // Set points of splines
  s_x.set_points(maps_s, maps_x);
  s_y.set_points(maps_s, maps_y);
  s_dx.set_points(maps_s, maps_dx);
  s_dy.set_points(maps_s, maps_dy);
}

//int Map::ClosestWaypoint(double x, double y) const
//{

//	double closestLen = 100000; //large number
//	int closestWaypoint = 0;

//	for(int i = 0; i < maps_x.size(); i++)
//	{
//		double map_x = maps_x[i];
//		double map_y = maps_y[i];
//		double dist = distance(x,y,map_x,map_y);
//		if(dist < closestLen)
//		{
//			closestLen = dist;
//			closestWaypoint = i;
//		}

//	}

//	return closestWaypoint;

//}

//int Map::NextWaypoint(double x, double y, double theta) const
//{

//	int closestWaypoint = ClosestWaypoint(x,y);

//	double map_x = maps_x[closestWaypoint];
//	double map_y = maps_y[closestWaypoint];

//	double heading = atan2((map_y-y),(map_x-x));

//	double angle = fabs(theta-heading);
//  angle = std::min(2*pi() - angle, angle);

//  if(angle > pi()/4)
//  {
//    closestWaypoint++;
//  if (closestWaypoint == maps_x.size())
//  {
//    closestWaypoint = 0;
//  }
//  }

//  return closestWaypoint;
//}

//// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//std::vector<double> Map::getFrenet(double x, double y, double theta) const
//{
//	int next_wp = NextWaypoint(x,y, theta);

//	int prev_wp;
//	prev_wp = next_wp-1;
//	if(next_wp == 0)
//	{
//		prev_wp  = maps_x.size()-1;
//	}

//	double n_x = maps_x[next_wp]-maps_x[prev_wp];
//	double n_y = maps_y[next_wp]-maps_y[prev_wp];
//	double x_x = x - maps_x[prev_wp];
//	double x_y = y - maps_y[prev_wp];

//	// find the projection of x onto n
//	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//	double proj_x = proj_norm*n_x;
//	double proj_y = proj_norm*n_y;

//	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

//	//see if d value is positive or negative by comparing it to a center point

//	double center_x = 1000-maps_x[prev_wp];
//	double center_y = 2000-maps_y[prev_wp];
//	double centerToPos = distance(center_x,center_y,x_x,x_y);
//	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

//	if(centerToPos <= centerToRef)
//	{
//		frenet_d *= -1;
//	}

//	// calculate s value
//	double frenet_s = 0;
//	for(int i = 0; i < prev_wp; i++)
//	{
//		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
//	}

//	frenet_s += distance(0,0,proj_x,proj_y);

//	return {frenet_s,frenet_d};

//}

//// Transform from Frenet s,d coordinates to Cartesian x,y
//std::vector<double> Map::getXY(double s, double d) const
//{
//	int prev_wp = -1;

//	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
//	{
//		prev_wp++;
//	}

//	int wp2 = (prev_wp+1)%maps_x.size();

//	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
//	// the x,y,s along the segment
//	double seg_s = (s-maps_s[prev_wp]);

//	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
//	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

//	double perp_heading = heading-pi()/2;

//	double x = seg_x + d*cos(perp_heading);
//	double y = seg_y + d*sin(perp_heading);

//	return {x,y};

//}

double Map::getCenterD(const int lane_id) const
{
  return LANE_WIDTH / 2.0 + lane_id * LANE_WIDTH;
}

int Map::getLaneId(const double d) const
{
  return static_cast<int>(d / LANE_WIDTH);
}

std::vector<double> Map::getXY(const double s, double d) const
{
  double s1 = std::fmod(s, MAX_S);
  double x = s_x(s1);
  double y = s_y(s1);
  double dx = s_dx(s1);
  double dy = s_dy(s1);
  return {x + d * dx, y + d * dy};
}

double Map::getYaw(const double s) const
{
  double dx = s_dx(s);
  double dy = s_dy(s);
  // dx and dy values define the unit normal vector pointing outward of the highway loop
  double theta = std::atan2(dy, dx) + M_PI_2;
  while (theta > M_PI) {
    theta -= 2 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2 * M_PI;
  }
  return theta;
}

std::vector<double> Map::getVelocityFrenet(const double vx, const double vy, const double s) const
{
  // Get lane yaw value
  double theta = getYaw(s);
  double vs = vx * std::cos(theta) + vy * std::sin(theta);
  double vd = vx * std::sin(theta) - vy * std::cos(theta);
  return {vs, vd};
}
