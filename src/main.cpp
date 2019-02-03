#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

const double SAFE_DISTANCE_AHEAD = 20;
const double SAFE_DISTANCE_BEHIND = -20;
const double MAX_VELOCITY = 49.5;
const double SWITCH_LANE_VELOCITY = 35.0;
const double MAX_SENSOR_RANGE = 300.0;

int GetLane(float d) {
	if (d < 4 && d > 0) {
		return 0;
	} else if (d < 8 && d > 4) {
		return 1;
	} else if (d < 12 && d > 8) {
		return 2;
	} else {
		return -1;
	}
}

vector<vector<json>> CarsInLane(json sensor_fusion) {
	vector<json> left;
	vector<json> middle;
	vector<json> right;

	sort(sensor_fusion.begin(), sensor_fusion.end(),
			[](const json &car1, const json &car2) {
			double car1_s = car1[5];
			double car2_s = car2[5];
			return car1_s < car2_s;
			});
	for (auto car: sensor_fusion) {
		switch(GetLane(car[6])) {
			case 0:
				left.push_back(car);
				break;
			case 1:
				middle.push_back(car);
				break;
			case 2:
				right.push_back(car);
				break;
		}
	}
	return {left, middle, right};
}

bool InGap(double position_s, double min_range, double max_range) {
	return (min_range < position_s && position_s < max_range);
}

bool IsGap(double car_s, double end_s, int car_lane, int other_cars_lane, vector<json> other_cars) {
	if (car_lane == other_cars_lane) { return false; };
	double min_safe_distance_ahead = end_s;
	double min_safe_distance_behind = car_s + SAFE_DISTANCE_BEHIND;

	bool car_in_gap = any_of(other_cars.begin(), other_cars.end(),
			[min_safe_distance_ahead, min_safe_distance_behind] (json other) {
			double other_s = other[5];
			return InGap(other_s, min_safe_distance_behind, min_safe_distance_ahead);
			});
	return !car_in_gap;
}

vector<double> LaneScores(double car_s, int car_lane, vector<json> &closest_car_in_each_lane) {
	vector<double> result{};
	for (int i = 0; i < 3; i++) {
		auto closest_car_in_lane = closest_car_in_each_lane[i];
		double score = 1.0;

		// Scoring is the distance to the closest car to us in a lane
		// divided by approximately the fartherest distance I've seen
		// on a sensor reading.
		if (closest_car_in_lane != NULL) {
			double other_s = closest_car_in_lane[5];
			score = (other_s - car_s) / MAX_SENSOR_RANGE;
		}

		// Boost lane score to keep the same lane.
		int distanceFromCurrentLane = abs(car_lane - i);
		if (distanceFromCurrentLane == 0) {
			score += 0.1;
		}
		result.push_back(score);
	}
	return result;
}

enum Action {
	change_left_2 = -2,
	change_left = -1,
	keep_lane = 0,
	change_right = 1,
	change_right_2 = 2,
	speed_up = 100,
	speed_down = 200,
};

double velocity(json other_car) {
	double vx = other_car[3];
	double vy = other_car[4];
	return sqrt(vx * vx + vy * vy);
}

bool TooClose(double car_s, double end_s, json other_car) {
	double other_s = other_car[5];
	bool ahead_of_us = other_s >= car_s;
	double distance = other_s - end_s;
	bool close_to_us = distance < SAFE_DISTANCE_AHEAD;
	bool result = ahead_of_us && close_to_us;

	LOG_IF(result, DEBUG) << "TOO CLOSE: " << result
		<< " car_s: " << car_s
		<< " other_s: " << other_s
		<< " ahead_of_us: " << ahead_of_us
		<< " distance_to_us: " << distance;
	return result;
}

json ClosestCarAheadInLane(double car_s, vector<json> cars_in_lane) {
	auto it = find_if(cars_in_lane.begin(), cars_in_lane.end(), [car_s] (json other_car) {
			double other_s = other_car[5];
			return other_s > car_s;
			});

	if (it != cars_in_lane.end()) {
		return *it;
	} else {
		return NULL;
	}
}

bool SafeToSwitchLanes(
		Action possibleAction,
		double car_v,
		double car_s,
		double end_s,
		int car_lane,
		int desired_lane,
		vector<vector<json>> cars_in_lanes) {
	bool safeSwitchSpeed = car_v >= SWITCH_LANE_VELOCITY;
	int incr = desired_lane < car_lane ?  -1 : 1;
	bool safeSpace = true;
	int lane_to_check = car_lane + incr;
	for (int i = 0; i < abs(possibleAction); i++) {
		safeSpace = safeSpace && IsGap(car_s, end_s, car_lane, desired_lane, cars_in_lanes[lane_to_check]);
		LOG(DEBUG) << "Checking lane " << lane_to_check << ": " << (safeSpace ? "true" : "false");
		lane_to_check += incr;
	}

	return safeSwitchSpeed && safeSpace;
}

int BestLane(vector<double> lane_scores) {
	auto it = max_element(lane_scores.begin(), lane_scores.end());
	return distance(lane_scores.begin(), it);
}

Action BestAction(double car_s, double end_s, double car_d, double ref_vel, double *desired_vel, vector<vector<json>> other_cars) {
	vector<json> closest_car_in_each_lane{};
	int myLane = GetLane(car_d);
	for (size_t i = 0; i < other_cars.size(); i++) {
		vector<json> cars_in_lane = other_cars[i];

		json closest_car_in_lane = ClosestCarAheadInLane(car_s, cars_in_lane);
		closest_car_in_each_lane.push_back(closest_car_in_lane);
	}

	auto lane_scores = LaneScores(car_s, myLane, closest_car_in_each_lane);

	json closest_car_in_current_lane = closest_car_in_each_lane[myLane];
	int best_lane = BestLane(lane_scores);

	LOG(DEBUG) << "BEST LANE: " << best_lane << " " << lane_scores;
	Action possibleLaneAction = static_cast<Action>( best_lane - myLane );

	// If there is a better lane to switch to and it's safe, try to switch.
	if (possibleLaneAction != Action::keep_lane && SafeToSwitchLanes(possibleLaneAction, ref_vel, car_s, end_s,
				myLane, best_lane, other_cars)) {
		return possibleLaneAction;
	}

	// Is the lane clear? reset speed or slow down;
	if (closest_car_in_current_lane != NULL) {
		double other_s = closest_car_in_current_lane[5];
		double distance = other_s - car_s;
		if (TooClose(car_s, end_s, closest_car_in_current_lane)) {
			double other_v = velocity(closest_car_in_current_lane);
			LOG(DEBUG) << "NO GAP SLOW DOWN TO MATCH: " << other_v;
			*desired_vel = other_v;
			return Action::speed_down;
		} else if (distance > 50) {
			*desired_vel = MAX_VELOCITY;
		}
	} else {
		*desired_vel = MAX_VELOCITY;
	}
	LOG(DEBUG) << "ref_vel: " << ref_vel << " desired_vel: " << *desired_vel;
	possibleLaneAction = ref_vel < *desired_vel ? Action::speed_up : Action::keep_lane;
	return possibleLaneAction;
}

int main(int argc, char* argv[]) {
	START_EASYLOGGINGPP(argc, argv);
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%datetime{%H:%m:%s} %level %line: %msg");
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	int lane = 1;
	double desired_vel = MAX_VELOCITY;
	double ref_vel = 1.0; // mph

	h.onMessage([&lane, &ref_vel, &desired_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
				uWS::OpCode opCode) {
			// "42" at the start of the message means there's a websocket message event.
			// The 4 signifies a websocket message
			// The 2 signifies a websocket event
			//auto sdata = string(data).substr(0, length);
			//cout << sdata << endl;
			if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
			auto j = json::parse(s);

			string event = j[0].get<string>();

			if (event == "telemetry") {
			// j[1] is the data JSON object
			// Main car's localization Data
			double car_x = j[1]["x"];
			double car_y = j[1]["y"];
			double car_s = j[1]["s"];
			double car_d = j[1]["d"];
			double car_yaw = j[1]["yaw"];
			double car_speed = j[1]["speed"];

			// Previous path data given to the Planner
			auto previous_path_x = j[1]["previous_path_x"];
			auto previous_path_y = j[1]["previous_path_y"];
			// Previous path's end s and d values
			double end_path_s = j[1]["end_path_s"];
			double end_path_d = j[1]["end_path_d"];

			// Sensor Fusion Data, a list of all other cars on the same side of the road.
			// The data format for each car is: [ id, x, y, vx, vy, s, d].
			// The id is a unique identifier for that car. The x, y values are
			// in global map coordinates, and the vx, vy values are the
			// velocity components, also in reference to the global map.
			// Finally s and d are the Frenet coordinates for that car.
			auto sensor_fusion = j[1]["sensor_fusion"];

			json msgJson;

			vector<double> ptsx;
			vector<double> ptsy;
			auto prev_size = previous_path_x.size();
			if (prev_size <= 0) { end_path_s = car_s; };

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			auto cars = CarsInLane(sensor_fusion);

			LOG(DEBUG) << "Car State: "
				<< " car_s: " << car_s
				<< " end_s: " << end_path_s
				<< " car_d: " << car_d
				<< " lane: " << GetLane(car_d);

			auto action = BestAction(car_s, end_path_s, car_d, ref_vel, &desired_vel, cars);
			switch(action) {
				case Action::change_left_2:
					ref_vel -= .224;
				case Action::change_left:
					if (GetLane(car_d) != lane) { break; }
					lane = max(lane - 1, 0);
					LOG(DEBUG) << "Action: Change Left " << lane;
					break;
				case Action::change_right_2:
					ref_vel -= .224;
				case Action::change_right:
					if (GetLane(car_d) != lane) { break; }
					lane = min(lane + 1, 2);
					LOG(DEBUG) << "Action: Change Right " << lane;
					break;
				case Action::keep_lane:
					LOG(DEBUG) << "Action: Keep Lane " << lane;
					break;
				case Action::speed_down:
					LOG(DEBUG) << "Action: Slow Down " << ref_vel;
					ref_vel -= .224;
					break;
				case Action::speed_up:
					LOG(DEBUG) << "Action: Speed Up " << ref_vel;
					ref_vel += .224;
					break;
				default:
					LOG(DEBUG) << "Action: " << action;
			}

			// If we are just starting out, generate some waypoints by
			// projecting our current trajectory backwards.
			if (prev_size < 2) {
				double prev_x = car_x - cos(car_yaw);
				double prev_y = car_y - sin(car_yaw);

				ptsx.push_back(prev_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_y);
				ptsy.push_back(car_y);
			} else {
				// Use previous points for new waypoints.
				ref_x = previous_path_x[prev_size - 1];
				ref_y = previous_path_y[prev_size - 1];

				double ref_x_prev = previous_path_x[prev_size - 2];
				double ref_y_prev = previous_path_y[prev_size - 2];

				ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);


				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}

			// Add waypoints 30, 60, 90m out.
			auto next_wp0 = getXY(end_path_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			auto next_wp1 = getXY(end_path_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			auto next_wp2 = getXY(end_path_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsy.push_back(next_wp0[1]);

			ptsx.push_back(next_wp1[0]);
			ptsy.push_back(next_wp1[1]);

			ptsx.push_back(next_wp2[0]);
			ptsy.push_back(next_wp2[1]);

			// Shift to car reference so that 0 angle is straight ahead.
			for (int i = 0; i < ptsx.size(); i++) {
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
				ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
			}

			tk::spline s;
			s.set_points(ptsx, ptsy);

			vector<double> next_x_vals;
			vector<double> next_y_vals;

			// Use the previously unconsumed path.
			for (int i = 0; i < prev_size; i++) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
			double x_add_on = 0;

			// Generate some points for the new path.
			for (int i=1; i <= 50 - prev_size; i++) {
				// Create points based on spline and space them apart
				// based on velocity.
				double N = (target_dist / (0.02 * ref_vel/2.24));
				double x_point = x_add_on + (target_x) / N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// Shift back from car reference to global XY.
				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}

			// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;

			auto msg = "42[\"control\","+ msgJson.dump()+"]";

			//this_thread::sleep_for(chrono::milliseconds(1000));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

			}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
			}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
				size_t, size_t) {
			const std::string s = "<h1>Hello world!</h1>";
			if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
			} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
			}
			});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
			std::cout << "Connected!!!" << std::endl;
			});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
				char *message, size_t length) {
			ws.close();
			std::cout << "Disconnected" << std::endl;
			});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
