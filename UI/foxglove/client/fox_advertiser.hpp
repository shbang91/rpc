#pragma once

#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/task.hpp"
#include <Eigen/Dense>
#include <foxglove/websocket/serialization.hpp>
#include <foxglove/websocket/websocket_client.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <list>

class FoxgloveTopicSubscriber {
public:
    FoxgloveTopicSubscriber();
    ~FoxgloveTopicSubscriber();

    void UpdateTopics();
    void Connect();

private:
    foxglove::SubscriptionId next_sub_id_;
    foxglove::Client<foxglove::WebSocketNoTls> client_;
};
