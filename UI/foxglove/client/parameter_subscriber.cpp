#include "parameter_subscriber.hpp"

FoxgloveParameterSubscriber::FoxgloveParameterSubscriber(std::unordered_map<std::string, int*> &parameters_map_int,
                                                         std::unordered_map<std::string, double*> &parameters_map_double,
                                                         const std::string url) {
  next_sub_id_ = 0;

  client_.setBinaryMessageHandler([&](const uint8_t* data, size_t dataLength) {
      if (static_cast<foxglove::BinaryOpcode>(data[0]) != foxglove::BinaryOpcode::MESSAGE_DATA) {
        return;
      }

      client_.getParameters({"n_steps","t_ss","t_ds"});
  });

  client_.setTextMessageHandler([&](const std::string& payload) {
      const nlohmann::basic_json msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();
      if (op != "advertise") {
        if (op == "parameterValues") {
          auto param_value = msg["parameters"].get<std::vector<foxglove::Parameter>>();
          for (const auto& param : param_value) {
            // assign new parameter value according to specified Map Type (after parsing from Foxglove)
            if (parameters_map_int.count(param.getName())) {
              double p_val = _ParseFoxgloveParameter(param);
              *parameters_map_int[param.getName()] = int(p_val);
            } else if (parameters_map_double.count(param.getName())) {
              double p_val = _ParseFoxgloveParameter(param);
              *parameters_map_double[param.getName()] = double(p_val);
            }
          }
        }
        return;
      }

      const auto channels = msg["channels"].get<std::vector<foxglove::Channel>>();
      std::vector<std::pair<foxglove::SubscriptionId, foxglove::ChannelId>> subscribePayload;

      for (const auto& channel : channels) {
        const auto subId = next_sub_id_++;
        subscribePayload.push_back({subId, channel.id});
      }

      if (!subscribePayload.empty()) {
        client_.subscribe(subscribePayload);
      }
  });

  const auto openHandler = [&](websocketpp::connection_hdl) {
      std::cout << "Connected to " << url << std::endl;
  };
  const auto closeHandler = [&](websocketpp::connection_hdl) {
      std::cout << "Connection closed" << std::endl;
  };

  client_.connect(url, openHandler, closeHandler);
}

FoxgloveParameterSubscriber::~FoxgloveParameterSubscriber() {
  client_.close();
}

void FoxgloveParameterSubscriber::UpdateParameters() {
  // add here parameters that need to be interpolated from current to desired value
}

double FoxgloveParameterSubscriber::_ParseFoxgloveParameter(const foxglove::Parameter& param) {
  if(param.getType() == foxglove::ParameterType::PARAMETER_INTEGER){
    int64_t p_val = param.getValue().getValue<int64_t>();
    return double(p_val);
  }
  else if(param.getType() == foxglove::ParameterType::PARAMETER_DOUBLE){
    return param.getValue().getValue<double>();
  }

  return 0.0;
}