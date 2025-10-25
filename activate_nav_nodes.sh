#!/bin/bash
echo "⏳ Waiting for nodes to start..."
sleep 10

echo "🚀 Activating navigation nodes..."

# Activate controller_server (usually already active)
ros2 lifecycle set /controller_server activate 2>/dev/null

# Activate planner_server (usually already active)
ros2 lifecycle set /planner_server activate 2>/dev/null

# Configure and activate bt_navigator
echo "  Activating bt_navigator..."
ros2 lifecycle set /bt_navigator configure
sleep 1
ros2 lifecycle set /bt_navigator activate
sleep 1

# Configure and activate behavior_server
echo "  Activating behavior_server..."
ros2 lifecycle set /behavior_server configure 2>/dev/null
sleep 1
ros2 lifecycle set /behavior_server activate 2>/dev/null
sleep 1

echo "✅ Done! Navigation ready."
echo ""
echo "Status check:"
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
echo ""
echo "Action server:"
ros2 action info /navigate_to_pose
