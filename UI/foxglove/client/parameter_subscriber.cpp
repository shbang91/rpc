#include "parameter_subscriber.hpp"

FoxgloveParameterSubscriber::FoxgloveParameterSubscriber(std::unordered_map<std::string, int*> &parameters_map_int,
                                                         std::unordered_map<std::string, double*> &parameters_map_double,
                                                         std::unordered_map<std::string, Task*> &parameters_map_task,
                                                         std::unordered_map<std::string, TaskHierarchyManager*> &parameters_map_hm,
                                                         const std::string url) {
  next_sub_id_ = 0;

  client_.setBinaryMessageHandler([&](const uint8_t* data, size_t dataLength) {
      if (static_cast<foxglove::BinaryOpcode>(data[0]) != foxglove::BinaryOpcode::MESSAGE_DATA) {
        return;
      }

      client_.getParameters({
          "n_steps", "t_ss", "t_ds",
          "torso_ori_task_weight", "torso_ori_task_kp", "torso_ori_task_kd",
          "foot_pos_task_weight", "foot_pos_task_weight_at_swing", "foot_pos_task_kp", "foot_pos_task_kd",
          "foot_ori_task_weight", "foot_ori_task_weight_at_swing", "foot_ori_task_kp", "foot_ori_task_kd"
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
            }     // ------ process as task from hierarchy manager --------
            else if (param.getName().find("foot_pos_task") != std::string::npos) {
              // loop through left and right feet managers to set new weight values
              _UpdateTaskHierarchyManager(param, "pos", parameters_map_hm);
            }
            else if (param.getName().find("foot_ori_task") != std::string::npos) {
              // loop through left and right feet managers to set new weight values
              _UpdateTaskHierarchyManager(param, "ori", parameters_map_hm);
            } else {  // ------ process as task --------
              // loop through all tasks to find the one that matches the parameter name
              for (auto &[name, task]: parameters_map_task) {
                if (param.getName().find(name) != std::string::npos) {
                  // set new weight, kp, or kd values
                  if (param.getName().find("weight") != std::string::npos) {
                    Eigen::Vector3d task_weights = _ParseFoxgloveParameterVec(param);   //ERROR IN HERE
                    task->SetWeight(task_weights);
                  } else if (param.getName().find("kp") != std::string::npos) {
                    Eigen::Vector3d task_kp = _ParseFoxgloveParameterVec(param);
                    task->SetKp(task_kp);
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
    int i_x = 0;
    for(const auto& pp_val : p_val){
        if (pp_val.getType() == foxglove::ParameterType::PARAMETER_DOUBLE){
            vec[i_x] = pp_val.getValue<double>();
        }
        else vec[i_x] = double(pp_val.getValue<int64_t>());
        i_x++;
    }
  }

  return vec;
}

void FoxgloveParameterSubscriber::_UpdateTaskHierarchyManager(const foxglove::Parameter &param,
                                                              const std::string &pos_or_ori,
                                                              std::unordered_map<std::string, TaskHierarchyManager*> &parameters_map_hm) {
  for (auto &[name_hm, task_hm]: parameters_map_hm) {
    if (name_hm.find(pos_or_ori) != std::string::npos) {
      // set new weight, kp, or kd values
      if (param.getName().find("weight_at_swing") != std::string::npos) {
        Eigen::Vector3d task_weights = _ParseFoxgloveParameterVec(param);
        task_hm->SetWeightMin(task_weights);
      } else if (param.getName().find("weight") != std::string::npos) {
        Eigen::Vector3d task_weights = _ParseFoxgloveParameterVec(param);
        task_hm->SetWeightMax(task_weights);
      } else if (param.getName().find("kp") != std::string::npos) {
        Eigen::Vector3d task_kp = _ParseFoxgloveParameterVec(param);
        task_hm->SetTaskKp(task_kp);
      } else if (param.getName().find("kd") != std::string::npos) {
        Eigen::Vector3d task_kd = _ParseFoxgloveParameterVec(param);
        task_hm->SetTaskKd(task_kd);
      }
    }
  }
}