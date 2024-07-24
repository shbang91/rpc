#pragma once

#include <list>
#include <foxglove/websocket/websocket_client.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/serialization.hpp>
#include <Eigen/Dense>
#include "controller/whole_body_controller/task.hpp"

class FoxgloveParameterSubscriber {
public:
  FoxgloveParameterSubscriber(std::unordered_map<std::string, int*> &parameters_map_int,
                              std::unordered_map<std::string, double*> &parameters_map_double,
                              std::unordered_map<std::string, Task*> &parameters_map_task,
                              const std::string url = "ws://localhost:8766");
  ~FoxgloveParameterSubscriber();

  void UpdateParameters();

private:
  double _ParseFoxgloveParameter(const foxglove::Parameter& param);
  Eigen::Vector3d _ParseFoxgloveParameterVec(const foxglove::Parameter& param);

private:
  foxglove::SubscriptionId next_sub_id_;
  foxglove::Client<foxglove::WebSocketNoTls> client_;
};
