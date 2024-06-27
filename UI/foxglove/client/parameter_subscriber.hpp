#pragma once

#include <foxglove/websocket/websocket_client.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/serialization.hpp>

class FoxgloveParameterSubscriber {
public:
  FoxgloveParameterSubscriber(std::unordered_map<std::string, double*> &parameters_map,
                              const std::string url = "ws://localhost:8766");
  ~FoxgloveParameterSubscriber();

  void UpdateParameters();

private:
  foxglove::SubscriptionId next_sub_id_;
  foxglove::Client<foxglove::WebSocketNoTls> client_;
};
