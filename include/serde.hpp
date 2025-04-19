#pragma once

#include <rclcpp/rclcpp.hpp>

namespace serde {
    template <typename T> T deserialize(const std::vector<uint8_t> &blob) {
        rmw_serialized_message_t cmsg = rmw_get_zero_initialized_serialized_message();
        rcutils_allocator_t alloc = rcutils_get_default_allocator();
        if (rmw_serialized_message_init(&cmsg, blob.size(), &alloc) != RMW_RET_OK) {
            std::cerr << "Failed to initialize serialized message" << std::endl;
        }
        memcpy(cmsg.buffer, blob.data(), blob.size());
        cmsg.buffer_length = blob.size();
        rclcpp::SerializedMessage serialized(cmsg);
        rclcpp::Serialization<T> serializer;
        T msg;
        serializer.deserialize_message(&serialized, &msg);
        return msg;
    }

    template <typename T> std::vector<uint8_t> serialize(const T &msg) {
        rclcpp::Serialization<T> serializer;
        rclcpp::SerializedMessage serialized;
        serializer.serialize_message(&msg, &serialized);
        auto rmw_msg = serialized.get_rcl_serialized_message();
        size_t len = rmw_msg.buffer_length;
        auto buf = reinterpret_cast<const uint8_t *>(rmw_msg.buffer);
        std::vector<uint8_t> blob(buf, buf + len);
        return blob;
    }
} // namespace farmbot
