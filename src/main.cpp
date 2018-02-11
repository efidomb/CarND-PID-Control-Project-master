#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int distance = 0;
int rounda = 0;
double best_err;
double err;
//double p[3] = { 0, 0, 0 };
double p[3] = { 2.77054, 0.0796149, 27.532 };
double dp[3] = { 1, 1, 1 };
bool run_A = true;
bool run_B = false;
bool run_C = true;
bool in_run = false;
bool first_best = true;
int psize_count = 0;
//int total_distance = 20000;
int total_distance = 200;
int num_stuck = 0;
bool stuck = false;
double prev_cte = 0;
double diff_cte;
bool B_init = true;
bool C_init = true;
bool enter = true;
double abs_cte;
bool pass = true;
double prev_err[3] = { 0, 0, 0 };
bool changed = false;
bool one_changed = false;
double p_best[3] = { 0, 0, 0 };
bool learning = false;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws) {
	std::string msg("42[\"reset\",{}]");
	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  //double p[3] = [0, 0, 0];
  //double dp[3] = [1, 1, 1];


  //pid.Init(0.2, 3, 0.004);
  //pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
		if (event == "telemetry") {
			if (learning) {
				std::cout << "distance, psize_count, round: " << distance << ", " << psize_count << ", " << rounda << std::endl;
				// j[1] is the data JSON object
				double cte = std::stod(j[1]["cte"].get<std::string>());
				double speed = std::stod(j[1]["speed"].get<std::string>());
				double angle = std::stod(j[1]["steering_angle"].get<std::string>());
				double steer_value;

				std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
				std::cout << "p: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
				pid.Init(p[0], p[1], p[2]);

				if (num_stuck > 10) {
					stuck = true;
				}
				distance += 1;
				if (distance <= total_distance && !stuck) {
					pid.int_cte += cte;
					abs_cte += cte * cte;
					if (pid.first_time) {
						diff_cte = 0;
						pid.first_time = false;
					}
					else {
						diff_cte = cte - prev_cte;
					}
					prev_cte = cte;
					if (fabs(cte) > 5) {
						num_stuck += 1;
					}
					std::cout << "diff_cte: " << diff_cte << std::endl;
					std::cout << "int_cte: " << pid.int_cte << std::endl;
					std::cout << "abs_cte: " << abs_cte << std::endl;
					std::cout << "best_err: " << best_err << std::endl;
					std::cout << "last_err: " << err << std::endl;
					std::cout << "present_err: " << abs_cte / distance << std::endl;
					std::cout << "num_stuck: " << num_stuck << std::endl;
					steer_value = -pid.Kp * cte - pid.Ki * pid.int_cte - pid.Kd * diff_cte;
					std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = 0.2;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
				else {
					//run_A = true;
					run_A = false;
					run_B = !run_B;
					run_C = !run_C;
					C_init = true;
					in_run = false;
					num_stuck = 0;
					stuck = false;
					prev_err[psize_count] = err;
					err = abs_cte / distance;
					distance = 0;
					pid.int_cte = 0;
					pid.first_time = true;
					abs_cte = 0;
					std::cout << "err: " << err << std::endl;
					std::cout << "run_B: " << run_B << std::endl;
					std::cout << "run_C: " << run_C << std::endl;
					reset_simulator(ws);
				}

				if (!run_A) {

					if (first_best) {
						first_best = false;
						best_err = err;
					}
					if (rounda < 30) { // should be 30 at list // should use sum_dp and tolerance
									   //double sum_dp = dp.sum();
						if (psize_count < (sizeof(p) / sizeof(p[0]))) {
							if (B_init) {
								B_init = false;
								in_run = true;
								enter = true;
								std::cout << "starting run B" << std::endl;
								p[psize_count] += dp[psize_count];
								PID pid;
								pid.Init(p[0], p[1], p[2]); // problematic? what about pid.int_cte? is he initialized?
							}
							std::cout << "enter: " << enter << std::endl;
							if (enter) {
								if (!in_run) {
									enter = false;
									std::cout << "did he enter here??" << std::endl;
									if (!run_B) {
										if (err < best_err) {
											best_err = err;
											dp[psize_count] *= 1.1;
											run_C = false;
											pass = false;
											one_changed = true;
											changed = true;
										}
										else {
											if (!changed) {
												if (err < prev_err[psize_count]) {
													dp[psize_count] *= 1.1;
													run_C = false;
													pass = false;
													one_changed = true;
													changed = true;
												}
												else {
													p[psize_count] -= 2 * dp[psize_count];
													run_C = true;
													if (!one_changed) {
														changed = false;
													}
													one_changed = false;
												}
											}
											else {
												p[psize_count] -= 2 * dp[psize_count];
												run_C = true;
												if (!one_changed) {
													changed = false;
												}
												one_changed = false;
											}
										}
									}
								}
							}
							std::cout << "run_C: " << run_C << std::endl;
							if (run_C) {
								std::cout << "after run C" << std::endl;
								if (C_init) {
									C_init = false;
									std::cout << "starting run C" << std::endl;
									PID pid;
									pid.Init(p[0], p[1], p[2]); // problematic?
									in_run = true;
								}
							}
							if (pass) {
								if (!in_run) {
									std::cout << "first after C" << std::endl;
									if (!run_C) {
										std::cout << "second after C" << std::endl;
										if (err < best_err) {
											best_err = err;
											dp[psize_count] *= 1.1;
											one_changed = true;
											changed = true;
										}
										else {
											if (!changed) {
												if (err < prev_err[psize_count]) {
													dp[psize_count] *= 1.1;
													one_changed = true;
													changed = true;
												}
												else {
													p[psize_count] += dp[psize_count];
													dp[psize_count] *= 0.9;
													if (!one_changed) {
														changed = false;
													}
													one_changed = false;
												}

											}
											else {
												p[psize_count] += dp[psize_count];
												dp[psize_count] *= 0.9;
												if (!one_changed) {
													changed = false;
												}
												one_changed = false;
											}
										}
									}
								}
							}


							if (!in_run) {
								psize_count += 1;
								std::cout << "psize_count: " << psize_count << std::endl;
								run_B = true;
								run_C = false;
								enter = true;
								B_init = true;
								C_init = false;
								pass = true;
							}

						}
						else {
							std::cout << "end round" << std::endl;
							rounda += 1;
							psize_count = 0;
						}
					}
				}
				else {
					std::string msg = "42[\"manual\",{}]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				double cte = std::stod(j[1]["cte"].get<std::string>());
				double speed = std::stod(j[1]["speed"].get<std::string>());
				double angle = std::stod(j[1]["steering_angle"].get<std::string>());
				double steer_value;

				pid.Init(p[0], p[1], p[2]);
				pid.UpdateError(cte);
				
				std::cout << "diff_cte: " << pid.diff_cte << std::endl;
				std::cout << "int_cte: " << pid.int_cte << std::endl;
				steer_value = pid.TotalError(cte);
				std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
				json msgJson;
				msgJson["steering_angle"] = steer_value;
				msgJson["throttle"] = 0.2;
				auto msg = "42[\"steer\"," + msgJson.dump() + "]";
				std::cout << msg << std::endl;
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
		else {
			std::string msg = "42[\"manual\",{}]";
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
		}
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
