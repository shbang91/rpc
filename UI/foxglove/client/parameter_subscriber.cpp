#include "parameter_subscriber.hpp"

FoxgloveParameterSubscriber::FoxgloveParameterSubscriber(std::unordered_map<std::string, int*> &parameters_int_map,
                                                         std::unordered_map<std::string, double*> &parameters_double_map,
                                                         const std::string url) {
  next_sub_id_ = 0;

  client_.setBinaryMessageHandler([&](const uint8_t* data, size_t dataLength) {
      if (static_cast<foxglove::BinaryOpcode>(data[0]) != foxglove::BinaryOpcode::MESSAGE_DATA) {
        return;
      }

      client_.getParameters({
          "n_steps","t_ss","t_ds",
          "1) weight"
      });
  });

  client_.setTextMessageHandler([&](const std::string& payload) {
      const nlohmann::basic_json msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();
      if (op != "advertise") {
        if (op == "parameterValues") {
          auto param_value = msg["parameters"].get<std::vector<foxglove::Parameter>>();
          for (const auto& param : param_value) {
              //handler needed to differentiate datatypes   :::   clean up later
            if(param.getType() == foxglove::ParameterType::PARAMETER_ARRAY){
                //std::cout << "type: ARRAY" << std::endl;
                auto p_arr = param.getValue().getValue<const std::vector<foxglove::ParameterValue>&>();
                std::cout << param.getName() << ": [ ";
                for(const auto& arr_parm : p_arr){
                    if(arr_parm.getType() == foxglove::ParameterType::PARAMETER_INTEGER){
                        auto p_val = arr_parm.getValue<int64_t>();
                        std::cout << p_val << " ";
                    }
                    else if(arr_parm.getType() == foxglove::ParameterType::PARAMETER_DOUBLE){
                        auto p_val = arr_parm.getValue<double>();
                        std::cout << p_val << " ";
                    }}
                std::cout << "]" << std::endl;
            }
            if(param.getType() == foxglove::ParameterType::PARAMETER_INTEGER){
                //std::cout << "type: INTEGER" << std::endl;
                auto p_val = param.getValue().getValue<int64_t>();
                std::cout << param.getName() << ": " << p_val << std::endl;
                *parameters_int_map[param.getName()] = p_val;
            }
            else if(param.getType() == foxglove::ParameterType::PARAMETER_DOUBLE){
                //std::cout << "type: DOUBLE" << std::endl;
                auto p_val = param.getValue().getValue<double>();
                std::cout << param.getName() << ": " << p_val << std::endl;
                //note -- compare pros/cons with template
                if(param.getName() == "n_steps"){   //replace with list in case we need exceptions for other int variables
                    int dp_val = round(p_val);
                    std::cout << param.getName() << " rounded to: " << dp_val << std::endl;
                    *parameters_int_map[param.getName()] = dp_val;
                }
                else *parameters_double_map[param.getName()] = p_val;
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