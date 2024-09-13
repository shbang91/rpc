#include "fox_advertiser.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>
#include <optional>
#include <vector>
#include <chrono>

constexpr char url[] = "ws://localhost:8765";

int cc = 0;

FoxgloveTopicSubscriber::FoxgloveTopicSubscriber() {
    const auto openHandler = [&](websocketpp::connection_hdl) {
        std::cout << "[FoxgloveTopicSubscriber] Connected to " << std::string(url) << std::endl;
    };
    const auto closeHandler = [&](websocketpp::connection_hdl) {
        std::cout << "[FoxgloveTopicSubscriber] Connection closed" << std::endl;
    };

    client_.connect(url, openHandler, closeHandler);
}

FoxgloveTopicSubscriber::~FoxgloveTopicSubscriber() { client_.close(); }

void FoxgloveTopicSubscriber::UpdateTopics() {

    // icp_est_chan_id = 4

    if(cc == 1){    // figure out how to do client advertise
        // Create and initialize the ClientAdvertisement variable "hii"
        foxglove::ClientAdvertisement client_advertisement;
        client_advertisement.channelId = 99; // need to figure out what this channel needs to be (either create topic or subscribe to current)
        client_advertisement.encoding = "json";
        client_advertisement.topic = "newicp_est";
        client_advertisement.schemaName = "newicp_est";
        // Convert the schema string to a vector of uint8_t
        const std::string schemaString = R"({"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"}}})";
        std::vector<uint8_t> schemaVector(schemaString.begin(), schemaString.end());
        client_advertisement.schema = schemaVector;
        // Create a vector and add the ClientAdvertisement object to it
        std::vector<foxglove::ClientAdvertisement> advertisements;
        advertisements.push_back(client_advertisement);

        try {
            // Advertise channel
            client_.advertise(advertisements);
            std::cout << "Channel advertised with ID: " << client_advertisement.channelId << std::endl;

            // Publish to channel
            const std::string jsonData = R"({"x": "69", "y": "69"})";   //need to figure out how to send this properly
            const uint8_t *buffer = reinterpret_cast<const uint8_t *>(jsonData.c_str());
            size_t size = std::strlen(jsonData.c_str());
            client_.publish(client_advertisement.channelId, buffer, size);
            std::cout << "Data published to channel ID: " << client_advertisement.channelId << std::endl;
        }
        catch (const std::exception& ex) {
            std::cerr << "Failed to advertise or publish: " << ex.what() << std::endl;
        }
    }
}

void FoxgloveTopicSubscriber::Connect() {
    if(cc == 0) {
        const auto openHandler = [&](websocketpp::connection_hdl) {
            std::cout << "[FoxgloveTopicSubscriber] Connected to " << std::string(url) << std::endl;
            cc = 1;
        };
        const auto closeHandler = [&](websocketpp::connection_hdl) {
            std::cout << "[FoxgloveTopicSubscriber] Connection closed" << std::endl;
        };
        client_.connect(url, openHandler, closeHandler);
    }
}