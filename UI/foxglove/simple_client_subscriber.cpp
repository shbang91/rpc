#include <foxglove/websocket/websocket_client.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/serialization.hpp>

#include <list>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <regex>

constexpr char DEFAULT_URI[] = "ws://localhost:8766";

std::atomic<bool> doAbort = false;

void signal_handler(int) {
  doAbort = true;
}

int main(int argc, char** argv) {
  const auto url = argc > 1 ? argv[1] : DEFAULT_URI;
  const auto topicRegex = argc > 2 ? std::regex(argv[2]) : std::regex(".*");
  foxglove::SubscriptionId nextSubId = 0;
  foxglove::Client<foxglove::WebSocketNoTls> client;

  client.setBinaryMessageHandler([&](const uint8_t* data, size_t dataLength) {
      if (static_cast<foxglove::BinaryOpcode>(data[0]) != foxglove::BinaryOpcode::MESSAGE_DATA) {
        return;
      }

      client.getParameters({"n_steps"});
  });

  client.setTextMessageHandler([&](const std::string& payload) {
      const nlohmann::basic_json msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();
      if (op != "advertise") {
        if (op == "parameterValues") {
          auto param_value = msg["parameters"].get<std::vector<foxglove::Parameter>>();
          for (const auto& param : param_value) {
            auto p_val = param.getValue().getValue<int64_t>();
            std::cout << param.getName() << ": " << p_val << std::endl;
          }
        }
        return;
      }

      const auto channels = msg["channels"].get<std::vector<foxglove::Channel>>();
      std::vector<std::pair<foxglove::SubscriptionId, foxglove::ChannelId>> subscribePayload;

      for (const auto& channel : channels) {
          const auto subId = nextSubId++;
          subscribePayload.push_back({subId, channel.id});
      }

      if (!subscribePayload.empty()) {
        client.subscribe(subscribePayload);
      }
  });

  const auto openHandler = [&](websocketpp::connection_hdl) {
      std::cout << "Connected to " << std::string(url) << std::endl;
  };
  const auto closeHandler = [&](websocketpp::connection_hdl) {
      std::cout << "Connection closed" << std::endl;
      doAbort = true;
  };

  client.connect(url, openHandler, closeHandler);
  std::signal(SIGINT, signal_handler);

  while (!doAbort) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  client.close();

  return EXIT_SUCCESS;
}
