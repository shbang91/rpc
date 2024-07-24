#include "parameter_subscriber.hpp"

FoxgloveParameterSubscriber::FoxgloveParameterSubscriber(std::unordered_map<std::string, int*> &parameters_map_int,
                                                         std::unordered_map<std::string, double*> &parameters_map_double,
                                                         std::unordered_map<std::string, Task*> &parameters_map_task,
                                                         const std::string url) {
  next_sub_id_ = 0;

  client_.setBinaryMessageHandler([&](const uint8_t* data, size_t dataLength) {
      if (static_cast<foxglove::BinaryOpcode>(data[0]) != foxglove::BinaryOpcode::MESSAGE_DATA) {
        return;
      }

      client_.getParameters({
          "n_steps", "t_ss", "t_ds",
          "torso_ori_task_weight", "torso_ori_task_kp", "torso_ori_task_kd",
      });
  });

  client_.setTextMessageHandler([&](const std::string& payload) {
      const nlohmann::basic_json msg = nlohmann::json::parse(payload);
      const auto& op = msg["op"].get<std::string>();
      if (op != "advertise") {
        if (op == "parameterValues") {
          auto param_value = msg["parameters"].get<std::vector<foxglove::Parameter>>();
          for (const auto& param : param_value) {
            // assign new parameter value according to specified Map Type (after parsing from Foxglove)
            // ------ process as int --------
            if (parameters_map_int.count(param.getName())) {
              double p_val = _ParseFoxgloveParameter(param);
              *parameters_map_int[param.getName()] = int(p_val);
            }         // ------ process as double --------
            else if (parameters_map_double.count(param.getName())) {
              double p_val = _ParseFoxgloveParameter(param);
              *parameters_map_double[param.getName()] = double(p_val);
            } else {  // ------ process as task --------
              // loop through all tasks to find the one that matches the parameter name
              for (auto &[name, task]: parameters_map_task) {
                if (param.getName().find(name) != std::string::npos) {
                  // set new weight, kp, or kd values
                  if (param.getName().find("weight") != std::string::npos) {
                    Eigen::Vector3d task_weights = _ParseFoxgloveParameterVec(param);
                    task->SetWeight(task_weights);
                  } else if (param.getName().find("kp") != std::string::npos) {
                    Eigen::Vector3d task_kp = _ParseFoxgloveParameterVec(param);
                  } else if (param.getName().find("kd") != std::string::npos) {
                    Eigen::Vector3d task_kd = _ParseFoxgloveParameterVec(param);
                    task->SetKd(task_kd);
                  }
                }
              }
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

Eigen::Vector3d FoxgloveParameterSubscriber::_ParseFoxgloveParameterVec(const foxglove::Parameter& param) {
  Eigen::Vector3d vec;
  if(param.getType() == foxglove::ParameterType::PARAMETER_ARRAY){
    auto p_val = param.getValue().getValue<std::vector<foxglove::ParameterValue>>();
    if (p_val[0].getType() == foxglove::ParameterType::PARAMETER_DOUBLE) {
      vec[0] = p_val[0].getValue<double>();
      vec[1] = p_val[1].getValue<double>();
      vec[2] = p_val[2].getValue<double>();
    } else {
      vec[0] = double(p_val[0].getValue<int64_t>());
      vec[1] = double(p_val[1].getValue<int64_t>());
      vec[2] = double(p_val[2].getValue<int64_t>());
    }
  }

  return vec;
}