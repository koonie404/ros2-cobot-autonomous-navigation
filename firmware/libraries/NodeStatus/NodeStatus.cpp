// =============================================
// NodeStatus.cpp
// =============================================
#include "NodeStatus.h"
#include <string.h>
#include <stdio.h>

void NodeStatus_begin(NodeStatus *ns, rcl_node_t *node, const char *node_name)
{
    ns->heartbeat_cnt = 0;
    ns->node_name     = node_name;

    rclc_publisher_init_default(
        &ns->heartbeat_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "esp32s3/heartbeat"
    );
    rclc_publisher_init_default(
        &ns->debug_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "esp32s3/debug"
    );

    ns->heartbeat_msg.data.data     = ns->heartbeat_buf;
    ns->heartbeat_msg.data.capacity = sizeof(ns->heartbeat_buf);
    ns->heartbeat_msg.data.size     = 0;

    ns->debug_msg.data.data         = ns->debug_buf;
    ns->debug_msg.data.capacity     = sizeof(ns->debug_buf);
    ns->debug_msg.data.size         = 0;
}

void NodeStatus_publishHeartbeat(NodeStatus *ns)
{
    ns->heartbeat_cnt++;
    snprintf(ns->heartbeat_buf, sizeof(ns->heartbeat_buf),
             "%s - %lu", ns->node_name, ns->heartbeat_cnt);
    ns->heartbeat_msg.data.size = strlen(ns->heartbeat_buf);
    rcl_publish(&ns->heartbeat_pub, &ns->heartbeat_msg, NULL);
}

void NodeStatus_publishDebug(NodeStatus *ns, const char *msg)
{
    strncpy(ns->debug_buf, msg, sizeof(ns->debug_buf) - 1);
    ns->debug_buf[sizeof(ns->debug_buf) - 1] = '\0';
    ns->debug_msg.data.size = strlen(ns->debug_buf);
    rcl_publish(&ns->debug_pub, &ns->debug_msg, NULL);
}
