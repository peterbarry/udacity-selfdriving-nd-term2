#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;


  mpc.solveTime_ms = 50; // assume solve time takes about 50ms to start, but update on each cycle.

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          double steer_value ;
          double throttle_value;
          int processing_delay=100;

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle_in = j[1]["throttle"];

          //Transform way points from global perspecitve to car relative.
          //cat at x=0, y=0, pointing straight.

          int num_points = ptsx.size();
          Eigen::VectorXd   way_x(num_points);
          Eigen::VectorXd   way_y(num_points);

          for (int i =0; i < num_points ; ++i )
          {
            double delta_x = ptsx[i] - px;
            double delta_y = ptsy[i] - py;
            way_x[i] = ((delta_x * cos( -psi )) - (delta_y * sin( -psi )));
            way_y[i] = ((delta_x * sin( -psi )) + (delta_y * cos( -psi )));
          }

          // Calcuate the polyfit line for the waypoints relative to car.
          auto coeffs = polyfit(way_x, way_y, 3);
          double cte  = polyeval(coeffs, 0);  // relative to car so px=0
          // epsi is -atan(coeffs1 + coeffs2*x + coeffs3* x^2)
          double epsi = -atan(coeffs[1]);


          const double Lf = 2.67;

          //Add latency for time when actuation occurs
          // calculate  average time for solve and remove from call if significant.


          double latency = 0.1; // 100ms
          // add the apprx time to solve cost function, to better predict future values.
          latency += (double)mpc.solveTime_ms / 1000.0;


          // The future calculations were borrowed from
          //https://github.com/mvirgo/MPC-Project

          double future_x = 0.0 + v * latency; // Along X plane.
          double future_y = 0.0; //assume along path.
          double future_psi = 0.0 + v * -steering_angle / Lf * latency;
          double future_v = v + throttle_in * latency;
          double future_cte = (cte + v * sin(epsi) * latency);
          double future_epsi = epsi + v * -steering_angle / Lf * latency;

          cout << "future_cte:" << future_cte << endl;
          cout << "future_x:" << future_x << endl;

          // Very crude attempt to cut corners
          double curve_indicator = polyeval(coeffs,100); // whats the value of y ahead./should be speed related

          future_y -= curve_indicator/10; // shift to side of curve

          cout << "curve_indicator:" << curve_indicator << "future_y:" << future_y << endl;
          if (future_y >= 1.0 )
            future_y = 1.0;
          if (future_y <= -1.0)
            future_y = -1.0;

          Eigen::VectorXd state(6);

          std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();




          //state << 0, 0, 0, v, cte, epsi;  // x,y and angle = 0 as we conveted from global space.
          state << future_x, future_y, future_psi, future_v, future_cte, future_epsi;

          cout << "STATE:" << state << endl << "********" << endl;

          auto solution = mpc.Solve(state, coeffs);
          steer_value = solution[0];
          throttle_value = solution[1];

          std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
          std::cout << "Solve Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" <<std::endl;
          //std::cout << "Solve Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;
          //std::cout << "Solve Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() <<std::endl;
          mpc.solveTime_ms = (double) std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -1 * steer_value/(deg2rad(25)*Lf);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i = 0; i < ptsx.size(); i += 1){
            mpc_x_vals.push_back(way_x[i]);
            mpc_y_vals.push_back(polyeval(coeffs, way_x[i]));
          }
          cout << "Coeffs:" << coeffs << endl;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for ( int i=0 ; i < ptsx.size(); ++i)
          {
            next_x_vals.push_back(way_x[i]);
            next_y_vals.push_back(way_y[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(processing_delay));
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
