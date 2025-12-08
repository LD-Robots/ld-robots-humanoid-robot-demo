#!/bin/bash
# ROS 2 LD Robots Humanoid Robot Demo - Terminal Shortcuts
# Source this file: source setup_shortcuts.bash

# Get project root directory
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ============================================================================
# Build & Source Shortcuts
# ============================================================================

alias cb='colcon build'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --packages-select'
alias cbv='colcon build --event-handlers console_direct+'
alias cs='source install/setup.bash'
alias ccs='colcon build && source install/setup.bash'
alias clean='rm -rf build/ install/ log/'
alias rebuild='rm -rf build/ install/ log/ && colcon build'

# Build specific package and source
cbps() {
    colcon build --packages-select "$1" && source install/setup.bash
}

# ============================================================================
# ROS 2 Command Shortcuts
# ============================================================================

# Topic commands
alias r2tl='ros2 topic list'
alias r2te='ros2 topic echo'
alias r2th='ros2 topic hz'
alias r2ti='ros2 topic info'

# Node commands
alias r2nl='ros2 node list'
alias r2ni='ros2 node info'

# Launch commands
alias r2l='ros2 launch'
alias r2r='ros2 run'

# Controller commands
alias r2cl='ros2 control list_controllers'
alias r2cs='ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers'

# ============================================================================
# Project-Specific Shortcuts
# ============================================================================

# Navigate to project
alias h_home='cd '"$PROJECT_ROOT"

# Launch shortcuts
alias h_gui='ros2 run gui_tools full_system_launcher'
alias h_full='ros2 launch humanoid_bringup full_system.launch.py'
alias h_sim='ros2 launch humanoid_bringup simulation.launch.py'
alias h_hw='ros2 launch humanoid_bringup hardware.launch.py'
alias h_minimal='ros2 launch humanoid_bringup minimal.launch.py'
alias h_moveit='ros2 launch humanoid_moveit_config demo.launch.py'
alias h_gazebo='ros2 launch humanoid_gazebo gazebo.launch.py'
alias h_rviz='ros2 launch humanoid_description display.launch.py'

# Launch with world selection
h_launch() {
    if [ -z "$1" ]; then
        echo "Usage: h_launch <world_name>"
        echo "Available worlds: empty.world, lab.world, outdoor.world, stairs.world"
        return 1
    fi
    ros2 launch humanoid_gazebo simulation.launch.py world:="$1"
}

# Test shortcuts
alias h_test='ros2 run demos simple_walk'
alias h_test_balance='ros2 run demos test_balance'
alias h_test_manipulation='ros2 run demos pick_place_demo'

# Controller monitoring
alias h_controllers='ros2 control list_controllers'
alias h_joints='ros2 topic echo /joint_states'
alias h_status='echo "=== Controllers ===" && ros2 control list_controllers && echo -e "\n=== Joint States ===" && ros2 topic hz /joint_states --once'

# Gazebo shortcuts
alias gz_gui='gz sim -g'
alias gz_topics='gz topic -l'
alias gz_models='gz model -l'

# ============================================================================
# RQT Tools (Debugging & Visualization)
# ============================================================================

# Launch RQT tools
alias h_rqt='rqt'
alias h_rqt_graph='rqt_graph'
alias h_rqt_plot='rqt_plot'
alias h_rqt_console='rqt_console'
alias h_rqt_reconfigure='rqt_reconfigure'
alias h_rqt_image='rqt_image_view'
alias h_rqt_tf='rqt_tf_tree'
alias h_rqt_bag='rqt_bag'

# PlotJuggler (superior plotting tool)
alias h_plotjuggler='plotjuggler'
alias h_pj='plotjuggler'  # Short alias

# PlotJuggler with ROS 2 streaming
h_plotjuggler_live() {
    echo "Starting PlotJuggler with ROS 2 streaming..."
    echo "In PlotJuggler: Streaming → Start ROS2 Topic Subscriber"
    plotjuggler &
}

# PlotJuggler with saved layout
h_plotjuggler_humanoid() {
    LAYOUT_FILE="$PROJECT_ROOT/config/plotjuggler_humanoid_layout.xml"
    if [ -f "$LAYOUT_FILE" ]; then
        echo "Loading humanoid layout from $LAYOUT_FILE"
        plotjuggler --layout "$LAYOUT_FILE" &
    else
        echo "⚠️  Layout file not found: $LAYOUT_FILE"
        echo "Starting PlotJuggler without layout..."
        echo "Tip: Save your layout to $LAYOUT_FILE for quick loading"
        plotjuggler &
    fi
}

# PlotJuggler with bag file
h_plotjuggler_bag() {
    if [ -z "$1" ]; then
        LATEST_BAG=$(ls -td "$PROJECT_ROOT/bags"/*/ 2>/dev/null | head -1)
        if [ -z "$LATEST_BAG" ]; then
            echo "No bag files found. Usage: h_plotjuggler_bag <bag_path>"
            return 1
        fi
        echo "Opening latest bag in PlotJuggler: $LATEST_BAG"
        plotjuggler --datafile "$LATEST_BAG" &
    else
        plotjuggler --datafile "$1" &
    fi
}

# Quick RQT launchers with common configs (legacy, use PlotJuggler instead)
h_plot_joints() {
    rqt_plot /joint_states/position &
}

h_plot_effort() {
    rqt_plot /joint_states/effort &
}

h_view_camera() {
    rqt_image_view /head_camera/color/image_raw &
}

h_view_depth() {
    rqt_image_view /head_camera/depth/image_raw &
}

# ============================================================================
# Log Management
# ============================================================================

# View build logs
alias h_logs='less log/latest_build/events.log'
alias h_build_log='cat log/latest_build/logger_all.log'

# Search for errors in logs
h_errors() {
    echo "=== Build Errors ==="
    grep -r "ERROR\|error:" log/latest_build/ 2>/dev/null | head -20
    echo ""
    echo "=== Runtime Errors ==="
    grep "ERROR" ~/.ros/log/*/rosout.log 2>/dev/null | tail -20
}

# Search for warnings
h_warnings() {
    echo "=== Build Warnings ==="
    grep -r "WARNING\|warning:" log/latest_build/ 2>/dev/null | head -20
}

# Clean old logs
alias h_clean_logs='rm -rf log/'
alias h_clean_ros_logs='rm -rf ~/.ros/log/*'

# View latest crash logs
h_crash_logs() {
    find ~/.ros/log -name "*.log" -mtime -1 -exec grep -l "Segmentation fault\|core dumped\|Fatal error" {} \;
}

# Tail ROS 2 daemon log
h_daemon_log() {
    tail -f ~/.ros/log/$(ls -t ~/.ros/log/ | head -1)/rosout.log
}

# ============================================================================
# Performance Profiling
# ============================================================================

# Topic bandwidth monitoring
h_bw() {
    if [ -z "$1" ]; then
        echo "Monitoring all major topics:"
        ros2 topic bw /joint_states &
        ros2 topic bw /camera/depth/points &
        ros2 topic bw /camera/color/image_raw &
        wait
    else
        ros2 topic bw "$1"
    fi
}

# Topic frequency monitoring
h_hz() {
    if [ -z "$1" ]; then
        echo "=== Topic Frequencies ==="
        echo "Joint States:"
        timeout 3 ros2 topic hz /joint_states
        echo ""
        echo "Camera Color:"
        timeout 3 ros2 topic hz /camera/color/image_raw
        echo ""
        echo "Point Cloud:"
        timeout 3 ros2 topic hz /camera/depth/points
    else
        ros2 topic hz "$1"
    fi
}

# CPU/Memory monitoring for ROS processes
h_perf() {
    echo "=== ROS 2 Process Performance ==="
    ps aux | grep -E "ros2|gz|rviz|moveit" | grep -v grep | awk '{printf "%-20s CPU: %5s%% MEM: %5s%% CMD: %s\n", $11, $3, $4, $0}' | sort -k3 -rn
}

# Watch resource usage
h_perf_watch() {
    watch -n 1 'ps aux | grep -E "ros2|gz|rviz|moveit" | grep -v grep | awk "{printf \"%-20s CPU: %5s%% MEM: %5s%%\n\", \$11, \$3, \$4}" | sort -k3 -rn | head -15'
}

# Network diagnostics
h_network() {
    echo "=== ROS 2 Network Diagnostics ==="
    echo "DDS Domain ID: ${ROS_DOMAIN_ID:-0}"
    echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-0}"
    echo ""
    echo "Active DDS participants:"
    ros2 daemon status
    echo ""
    echo "Topic bandwidth:"
    timeout 5 ros2 topic bw /joint_states 2>/dev/null || echo "No data on /joint_states"
}

# ============================================================================
# ROS 2 Bag Recording & Playback
# ============================================================================

# Quick record all topics
h_record_all() {
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    mkdir -p "$PROJECT_ROOT/bags"
    echo "Recording to bags/humanoid_${TIMESTAMP}"
    ros2 bag record -a -o "$PROJECT_ROOT/bags/humanoid_${TIMESTAMP}"
}

# Record specific topics
h_record() {
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    mkdir -p "$PROJECT_ROOT/bags"
    echo "Recording selected topics to bags/humanoid_${TIMESTAMP}"
    ros2 bag record -o "$PROJECT_ROOT/bags/humanoid_${TIMESTAMP}" \
        /joint_states \
        /humanoid_controller/state \
        /head_camera/color/image_raw \
        /head_camera/depth/points \
        /imu/data
}

# Record joint states only (lightweight)
h_record_joints() {
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    mkdir -p "$PROJECT_ROOT/bags"
    echo "Recording joint states to bags/joints_${TIMESTAMP}"
    ros2 bag record -o "$PROJECT_ROOT/bags/joints_${TIMESTAMP}" /joint_states
}

# Play latest bag
h_play() {
    LATEST_BAG=$(ls -td "$PROJECT_ROOT/bags"/*/ 2>/dev/null | head -1)
    if [ -z "$LATEST_BAG" ]; then
        echo "No bag files found in $PROJECT_ROOT/bags/"
        return 1
    fi
    echo "Playing: $LATEST_BAG"
    ros2 bag play "$LATEST_BAG"
}

# List available bags
h_list_bags() {
    echo "=== Available Bag Files ==="
    ls -lht "$PROJECT_ROOT/bags" 2>/dev/null || echo "No bags directory found"
}

# Bag info
h_bag_info() {
    if [ -z "$1" ]; then
        LATEST_BAG=$(ls -td "$PROJECT_ROOT/bags"/*/ 2>/dev/null | head -1)
        if [ -z "$LATEST_BAG" ]; then
            echo "No bag files found. Usage: h_bag_info <bag_path>"
            return 1
        fi
        ros2 bag info "$LATEST_BAG"
    else
        ros2 bag info "$1"
    fi
}

# ============================================================================
# Parameter Management
# ============================================================================

# Dump all parameters
h_params_dump() {
    if [ -z "$1" ]; then
        echo "Usage: h_params_dump <node_name>"
        echo "Available nodes:"
        ros2 node list
        return 1
    fi
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    mkdir -p "$PROJECT_ROOT/params"
    ros2 param dump "$1" --output-dir "$PROJECT_ROOT/params" --print
    echo "Parameters saved to params/"
}

# List all parameters for a node
h_params_list() {
    if [ -z "$1" ]; then
        echo "Usage: h_params_list <node_name>"
        echo "Available nodes:"
        ros2 node list
        return 1
    fi
    ros2 param list "$1"
}

# Get specific parameter
h_params_get() {
    if [ -z "$1" ] || [ -z "$2" ]; then
        echo "Usage: h_params_get <node_name> <param_name>"
        return 1
    fi
    ros2 param get "$1" "$2"
}

# Set parameter
h_params_set() {
    if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
        echo "Usage: h_params_set <node_name> <param_name> <value>"
        return 1
    fi
    ros2 param set "$1" "$2" "$3"
}

# ============================================================================
# Multi-Terminal Workflows (tmux)
# ============================================================================

# Check if tmux is installed
_check_tmux() {
    if ! command -v tmux &> /dev/null; then
        echo "❌ tmux is not installed. Install with: sudo apt install tmux"
        return 1
    fi
    return 0
}

# Full development session
h_dev_session() {
    _check_tmux || return 1

    SESSION="humanoid_dev"

    # Kill existing session if it exists
    tmux kill-session -t $SESSION 2>/dev/null

    # Create new session
    tmux new-session -d -s $SESSION -n "main" -c "$PROJECT_ROOT"

    # Window 0: System launcher
    tmux send-keys -t $SESSION:0 "echo 'Launch system with: h_gui or h_full'" C-m

    # Window 1: Monitor
    tmux new-window -t $SESSION:1 -n "monitor" -c "$PROJECT_ROOT"
    tmux send-keys -t $SESSION:1 "sleep 10 && h_monitor" C-m

    # Window 2: Logs
    tmux new-window -t $SESSION:2 -n "logs" -c "$PROJECT_ROOT"
    tmux send-keys -t $SESSION:2 "echo 'Use: h_errors, h_daemon_log, etc.'" C-m

    # Window 3: Development
    tmux new-window -t $SESSION:3 -n "dev" -c "$PROJECT_ROOT"

    # Attach to session
    tmux attach -t $SESSION
}

# Quick monitoring session
h_monitor_session() {
    _check_tmux || return 1

    SESSION="humanoid_monitor"
    tmux kill-session -t $SESSION 2>/dev/null

    tmux new-session -d -s $SESSION -n "status" -c "$PROJECT_ROOT"

    # Pane 0: Controllers
    tmux send-keys -t $SESSION:0.0 "watch -n 1 'ros2 control list_controllers'" C-m

    # Pane 1: Joint states
    tmux split-window -t $SESSION:0 -h -c "$PROJECT_ROOT"
    tmux send-keys -t $SESSION:0.1 "h_watch" C-m

    # Pane 2: Performance
    tmux split-window -t $SESSION:0.0 -v -c "$PROJECT_ROOT"
    tmux send-keys -t $SESSION:0.2 "h_perf_watch" C-m

    tmux attach -t $SESSION
}

# Kill all tmux sessions
h_kill_sessions() {
    tmux kill-session -t humanoid_dev 2>/dev/null
    tmux kill-session -t humanoid_monitor 2>/dev/null
    echo "✅ All humanoid tmux sessions killed"
}

# ============================================================================
# Code Generation Templates
# ============================================================================

# Create new ROS 2 Python node
h_new_node() {
    if [ -z "$1" ] || [ -z "$2" ]; then
        echo "Usage: h_new_node <package_name> <node_name>"
        echo "Example: h_new_node humanoid_control my_controller_node"
        return 1
    fi

    PKG=$1
    NODE=$2
    NODE_FILE="$PROJECT_ROOT/src/$PKG/${PKG}/${NODE}.py"

    if [ -f "$NODE_FILE" ]; then
        echo "❌ Node already exists: $NODE_FILE"
        return 1
    fi

    mkdir -p "$PROJECT_ROOT/src/$PKG/${PKG}"

    cat > "$NODE_FILE" << 'NODEEOF'
#!/usr/bin/env python3
"""
NODE_NAME - Description

Author: Your Name
Date: DATE
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NODE_CLASSNode(Node):
    """NODE_CLASS node implementation."""

    def __init__(self):
        super().__init__('NODE_NAME')

        # Parameters
        self.declare_parameter('rate', 10.0)

        # Publishers
        self.publisher_ = self.create_publisher(String, 'output', 10)

        # Subscribers
        self.subscription_ = self.create_subscription(
            String, 'input', self.listener_callback, 10
        )

        # Timers
        rate = self.get_parameter('rate').value
        self.timer_ = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('NODE_CLASS node started')

    def listener_callback(self, msg):
        """Handle incoming messages."""
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        """Periodic callback."""
        msg = String()
        msg.data = 'Hello from NODE_NAME'
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NODE_CLASSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
NODEEOF

    # Replace placeholders
    sed -i "s/NODE_NAME/$NODE/g" "$NODE_FILE"
    sed -i "s/NODE_CLASS/${NODE^}/g" "$NODE_FILE"
    sed -i "s/DATE/$(date +%Y-%m-%d)/g" "$NODE_FILE"

    chmod +x "$NODE_FILE"

    echo "✅ Created node: $NODE_FILE"
    echo "   Add to setup.py entry_points:"
    echo "   '$NODE = ${PKG}.${NODE}:main',"
}

# Create new launch file
h_new_launch() {
    if [ -z "$1" ] || [ -z "$2" ]; then
        echo "Usage: h_new_launch <package_name> <launch_name>"
        echo "Example: h_new_launch humanoid_control my_system"
        return 1
    fi

    PKG=$1
    LAUNCH=$2
    LAUNCH_FILE="$PROJECT_ROOT/src/$PKG/launch/${LAUNCH}.launch.py"

    if [ -f "$LAUNCH_FILE" ]; then
        echo "❌ Launch file already exists: $LAUNCH_FILE"
        return 1
    fi

    mkdir -p "$PROJECT_ROOT/src/$PKG/launch"

    cat > "$LAUNCH_FILE" << 'LAUNCHEOF'
"""
LAUNCH_NAME Launch File

Description: Add description here

Author: Your Name
Date: DATE
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodes
    example_node = Node(
        package='PACKAGE_NAME',
        executable='node_name',
        name='node_name',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg='Launching LAUNCH_NAME...'),
        example_node,
    ])
LAUNCHEOF

    # Replace placeholders
    sed -i "s/LAUNCH_NAME/$LAUNCH/g" "$LAUNCH_FILE"
    sed -i "s/PACKAGE_NAME/$PKG/g" "$LAUNCH_FILE"
    sed -i "s/DATE/$(date +%Y-%m-%d)/g" "$LAUNCH_FILE"

    echo "✅ Created launch file: $LAUNCH_FILE"
}

# ============================================================================
# Documentation Helpers
# ============================================================================

# Quick access to documentation
alias h_docs='cd '"$PROJECT_ROOT"'/docs && ls -lh'
alias h_readme='less '"$PROJECT_ROOT"'/CLAUDE.md'

# Show package info
h_pkg_info() {
    if [ -z "$1" ]; then
        echo "Usage: h_pkg_info <package_name>"
        echo "Available packages:"
        ls "$PROJECT_ROOT/src" -1
        return 1
    fi
    ros2 pkg prefix "$1" 2>/dev/null && \
    echo "" && \
    echo "Package contents:" && \
    ros2 pkg executables "$1"
}

# List all ROS 2 interfaces in package
h_interfaces() {
    if [ -z "$1" ]; then
        echo "Usage: h_interfaces <package_name>"
        return 1
    fi
    echo "=== Messages ==="
    ros2 interface list | grep "^$1" | grep "/msg/"
    echo ""
    echo "=== Services ==="
    ros2 interface list | grep "^$1" | grep "/srv/"
    echo ""
    echo "=== Actions ==="
    ros2 interface list | grep "^$1" | grep "/action/"
}

# Show interface definition
h_interface_show() {
    if [ -z "$1" ]; then
        echo "Usage: h_interface_show <interface_name>"
        echo "Example: h_interface_show sensor_msgs/msg/JointState"
        return 1
    fi
    ros2 interface show "$1"
}

# ============================================================================
# Dependency Management
# ============================================================================

# Install dependencies
h_deps_install() {
    cd "$PROJECT_ROOT" && \
    rosdep install --from-paths src --ignore-src -r -y
}

# Check for missing dependencies
h_deps_check() {
    cd "$PROJECT_ROOT" && \
    rosdep check --from-paths src --ignore-src
}

# Update rosdep database
h_deps_update() {
    rosdep update
}

# List package dependencies
h_deps_list() {
    if [ -z "$1" ]; then
        echo "Usage: h_deps_list <package_name>"
        echo "Available packages:"
        ls "$PROJECT_ROOT/src" -1
        return 1
    fi

    PKG_XML="$PROJECT_ROOT/src/$1/package.xml"
    if [ ! -f "$PKG_XML" ]; then
        echo "❌ Package not found: $1"
        return 1
    fi

    echo "=== Dependencies for $1 ==="
    grep -E "<depend>|<build_depend>|<exec_depend>" "$PKG_XML" | sed 's/.*<[^>]*>\(.*\)<.*/  - \1/'
}

# ============================================================================
# Workspace Backup & Restore
# ============================================================================

# Backup configuration files
h_backup_config() {
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    BACKUP_DIR="$PROJECT_ROOT/backups/config_${TIMESTAMP}"
    mkdir -p "$BACKUP_DIR"

    echo "Backing up configuration files to $BACKUP_DIR"

    # Find and copy all config files
    find "$PROJECT_ROOT/src" -name "*.yaml" -o -name "*.rviz" -o -name "*.srdf" | \
        while read file; do
            mkdir -p "$BACKUP_DIR/$(dirname ${file#$PROJECT_ROOT/})"
            cp "$file" "$BACKUP_DIR/${file#$PROJECT_ROOT/}"
        done

    echo "✅ Backup complete: $BACKUP_DIR"
    du -sh "$BACKUP_DIR"
}

# List backups
h_list_backups() {
    if [ -d "$PROJECT_ROOT/backups" ]; then
        echo "=== Available Backups ==="
        ls -lht "$PROJECT_ROOT/backups"
    else
        echo "No backups found"
    fi
}

# ============================================================================
# Diagnostic Functions
# ============================================================================

# Check if system is ready
h_check() {
    echo "=== System Health Check ==="
    echo ""
    echo "Controllers:"
    ros2 control list_controllers 2>/dev/null || echo "  ❌ Controller manager not running"
    echo ""
    echo "Active nodes:"
    ros2 node list 2>/dev/null | head -10 || echo "  ❌ No ROS 2 nodes running"
    echo ""
    echo "Joint states:"
    timeout 2 ros2 topic hz /joint_states 2>/dev/null || echo "  ❌ Joint states not publishing"
}

# Watch joint states
h_watch() {
    ros2 topic echo /joint_states --field name,position
}

# Monitor controller performance
h_monitor() {
    watch -n 0.5 'ros2 control list_controllers && echo "" && ros2 topic hz /joint_states --once'
}

# ============================================================================
# Development Helpers
# ============================================================================

# Quick rebuild of specific package
h_rebuild() {
    if [ -z "$1" ]; then
        echo "Usage: h_rebuild <package_name>"
        echo "Available packages: humanoid_control, humanoid_gazebo, humanoid_moveit_config, etc."
        return 1
    fi
    cd "$PROJECT_ROOT" && \
    colcon build --packages-select "$1" --symlink-install && \
    source install/setup.bash && \
    echo "✅ Package $1 rebuilt and sourced"
}

# Show available commands
h_help() {
    echo "=== LD Robots Humanoid Robot Demo - Terminal Shortcuts ==="
    echo ""
    echo "Build & Source:"
    echo "  cb                - colcon build"
    echo "  cbs               - colcon build --symlink-install"
    echo "  cbp <pkg>         - colcon build --packages-select"
    echo "  cs                - source install/setup.bash"
    echo "  ccs               - build and source"
    echo "  cbps <pkg>        - build package and source"
    echo "  clean             - remove build/install/log"
    echo "  rebuild           - clean build"
    echo ""
    echo "Launch System:"
    echo "  h_gui           - Launch GUI system launcher (RECOMMENDED)"
    echo "  h_full          - Launch full system (all controllers + sensors)"
    echo "  h_sim           - Launch simulation environment"
    echo "  h_hw            - Launch hardware interface"
    echo "  h_minimal       - Launch minimal system (basic controllers only)"
    echo "  h_launch <w>    - Launch with specific world"
    echo "  h_moveit        - Launch MoveIt planning"
    echo "  h_gazebo        - Launch Gazebo simulation"
    echo "  h_rviz          - Launch RViz visualization"
    echo ""
    echo "Testing:"
    echo "  h_test               - Run simple walk demo"
    echo "  h_test_balance       - Test balance controller"
    echo "  h_test_manipulation  - Test pick and place"
    echo ""
    echo "Monitoring:"
    echo "  h_check         - System health check"
    echo "  h_status        - Controller and topic status"
    echo "  h_controllers   - List active controllers"
    echo "  h_joints        - Echo joint states"
    echo "  h_watch         - Watch joint positions"
    echo "  h_monitor       - Live controller monitor"
    echo ""
    echo "Visualization Tools (type 'h_help_rqt' for more):"
    echo "  h_plotjuggler      - Launch PlotJuggler (RECOMMENDED)"
    echo "  h_pj               - PlotJuggler short alias"
    echo "  h_plotjuggler_live - PlotJuggler with live ROS 2 streaming"
    echo "  h_plotjuggler_bag  - Open bag file in PlotJuggler"
    echo "  h_rqt_graph        - Node/topic graph visualization"
    echo "  h_view_camera      - View RGB camera"
    echo "  h_view_depth       - View depth camera"
    echo ""
    echo "Performance & Profiling (type 'h_help_perf' for more):"
    echo "  h_perf          - Show CPU/memory usage"
    echo "  h_perf_watch    - Live resource monitoring"
    echo "  h_hz [topic]    - Topic frequency"
    echo "  h_bw [topic]    - Topic bandwidth"
    echo "  h_network       - Network diagnostics"
    echo ""
    echo "Log Management (type 'h_help_logs' for more):"
    echo "  h_logs          - View build logs"
    echo "  h_errors        - Search for errors"
    echo "  h_warnings      - Search for warnings"
    echo "  h_daemon_log    - Tail ROS 2 daemon log"
    echo "  h_clean_logs    - Clean build logs"
    echo ""
    echo "ROS 2 Bags (type 'h_help_bags' for more):"
    echo "  h_record        - Record selected topics"
    echo "  h_record_all    - Record all topics"
    echo "  h_record_joints - Record joint states only"
    echo "  h_play          - Play latest bag"
    echo "  h_list_bags     - List all bags"
    echo "  h_bag_info      - Show bag info"
    echo ""
    echo "Parameters (type 'h_help_params' for more):"
    echo "  h_params_dump <node>  - Dump node parameters"
    echo "  h_params_list <node>  - List node parameters"
    echo "  h_params_get <n> <p>  - Get parameter value"
    echo "  h_params_set <n> <p> <v> - Set parameter"
    echo ""
    echo "Multi-Terminal (tmux) (type 'h_help_tmux' for more):"
    echo "  h_dev_session   - Start full development session"
    echo "  h_monitor_session - Start monitoring session"
    echo "  h_kill_sessions - Kill all tmux sessions"
    echo ""
    echo "Code Generation (type 'h_help_codegen' for more):"
    echo "  h_new_node <pkg> <name>   - Create new ROS 2 node"
    echo "  h_new_launch <pkg> <name> - Create new launch file"
    echo ""
    echo "Documentation (type 'h_help_docs' for more):"
    echo "  h_docs               - Navigate to docs folder"
    echo "  h_readme             - View CLAUDE.md"
    echo "  h_pkg_info <pkg>     - Show package info"
    echo "  h_interfaces <pkg>   - List package interfaces"
    echo ""
    echo "Dependencies (type 'h_help_deps' for more):"
    echo "  h_deps_install  - Install all dependencies"
    echo "  h_deps_check    - Check for missing dependencies"
    echo "  h_deps_list <pkg> - List package dependencies"
    echo ""
    echo "Backup & Restore (type 'h_help_backup' for more):"
    echo "  h_backup_config - Backup configuration files"
    echo "  h_list_backups  - List available backups"
    echo ""
    echo "Development:"
    echo "  h_rebuild <pkg> - Rebuild specific package"
    echo "  h_home          - Navigate to project root"
    echo ""
    echo "ROS 2 Commands:"
    echo "  r2tl, r2te, r2th, r2ti - topic list/echo/hz/info"
    echo "  r2nl, r2ni             - node list/info"
    echo "  r2l, r2r               - launch/run"
    echo "  r2cl, r2cs             - controller list/status"
    echo ""
    echo "Gazebo:"
    echo "  gz_gui            - Launch Gazebo GUI viewer"
    echo "  gz_topics         - List Gazebo topics"
    echo "  gz_models         - List Gazebo models"
    echo ""
    echo "For detailed help on specific topics, use:"
    echo "  h_help_rqt, h_help_perf, h_help_logs, h_help_bags,"
    echo "  h_help_params, h_help_tmux, h_help_codegen, h_help_docs,"
    echo "  h_help_deps, h_help_backup"
    echo ""
    echo "Quick tips:"
    echo "  - Use 'h_home' to return to project root"
    echo "  - Use 'h_rviz' to visualize the robot"
    echo "  - Use 'h_sim' to start the simulation"
    echo "  - Use 'h_pj' for PlotJuggler (best for data visualization)"
}

# Detailed help sections
h_help_rqt() {
    echo "=== RQT Tools - Debugging & Visualization ==="
    echo ""
    echo "PlotJuggler (RECOMMENDED for plotting):"
    echo "  h_plotjuggler         - Launch PlotJuggler"
    echo "  h_pj                  - Short alias for PlotJuggler"
    echo "  h_plotjuggler_live    - PlotJuggler with ROS 2 live streaming"
    echo "  h_plotjuggler_arm     - Load saved arm layout (if exists)"
    echo "  h_plotjuggler_bag [path] - Open bag file in PlotJuggler"
    echo ""
    echo "PlotJuggler Features:"
    echo "  - Superior time-series visualization vs rqt_plot"
    echo "  - Pan, zoom, multi-axis plots with ease"
    echo "  - Live ROS 2 streaming and bag file replay"
    echo "  - Save/load custom layouts"
    echo "  - Data transformations and statistics"
    echo ""
    echo "Install PlotJuggler:"
    echo "  sudo apt install ros-jazzy-plotjuggler-ros"
    echo ""
    echo "Launch RQT Tools:"
    echo "  h_rqt              - Launch main RQT interface"
    echo "  h_rqt_graph        - Node/topic graph visualization"
    echo "  h_rqt_plot         - Real-time data plotting (basic)"
    echo "  h_rqt_console      - Log message console"
    echo "  h_rqt_reconfigure  - Dynamic parameter reconfiguration"
    echo "  h_rqt_image        - Image viewer"
    echo "  h_rqt_tf           - TF tree visualization"
    echo "  h_rqt_bag          - Bag file viewer"
    echo ""
    echo "Quick Launchers:"
    echo "  h_plot_joints      - Plot all 6 joint positions (rqt_plot)"
    echo "  h_plot_effort      - Plot all 6 joint efforts (rqt_plot)"
    echo "  h_view_camera      - View RGB camera feed"
    echo "  h_view_depth       - View depth camera feed"
}

h_help_perf() {
    echo "=== Performance Profiling & Network Diagnostics ==="
    echo ""
    echo "Resource Monitoring:"
    echo "  h_perf             - Show CPU/memory usage for ROS processes"
    echo "  h_perf_watch       - Live resource monitoring (updates every 1s)"
    echo ""
    echo "Topic Performance:"
    echo "  h_hz               - Show frequency of all major topics"
    echo "  h_hz <topic>       - Show frequency of specific topic"
    echo "  h_bw               - Show bandwidth of all major topics"
    echo "  h_bw <topic>       - Show bandwidth of specific topic"
    echo ""
    echo "Network Diagnostics:"
    echo "  h_network          - DDS domain, participants, bandwidth"
}

h_help_logs() {
    echo "=== Log Management ==="
    echo ""
    echo "View Logs:"
    echo "  h_logs             - View build event logs"
    echo "  h_build_log        - View full build log"
    echo "  h_daemon_log       - Tail ROS 2 daemon log (live)"
    echo ""
    echo "Search Logs:"
    echo "  h_errors           - Search for build and runtime errors"
    echo "  h_warnings         - Search for build warnings"
    echo "  h_crash_logs       - Find recent crash logs"
    echo ""
    echo "Clean Logs:"
    echo "  h_clean_logs       - Remove build logs"
    echo "  h_clean_ros_logs   - Remove ROS 2 runtime logs"
}

h_help_bags() {
    echo "=== ROS 2 Bag Recording & Playback ==="
    echo ""
    echo "Recording:"
    echo "  h_record           - Record selected topics (joints, camera, feedback)"
    echo "  h_record_all       - Record ALL topics (large file!)"
    echo "  h_record_joints    - Record joint states only (lightweight)"
    echo ""
    echo "Playback:"
    echo "  h_play             - Play latest bag file"
    echo "  h_play <path>      - Play specific bag file"
    echo ""
    echo "Info:"
    echo "  h_list_bags        - List all recorded bags"
    echo "  h_bag_info         - Show info for latest bag"
    echo "  h_bag_info <path>  - Show info for specific bag"
    echo ""
    echo "Bags are saved to: $PROJECT_ROOT/bags/"
}

h_help_params() {
    echo "=== Parameter Management ==="
    echo ""
    echo "Node Parameters:"
    echo "  h_params_dump <node>       - Dump all parameters to YAML file"
    echo "  h_params_list <node>       - List all parameters for node"
    echo "  h_params_get <node> <param> - Get specific parameter value"
    echo "  h_params_set <node> <param> <value> - Set parameter value"
    echo ""
    echo "Example:"
    echo "  h_params_list /controller_manager"
    echo "  h_params_get /controller_manager update_rate"
    echo "  h_params_set /perception_node processing_rate 2.0"
    echo ""
    echo "Dumped parameters are saved to: $PROJECT_ROOT/params/"
}

h_help_tmux() {
    echo "=== Multi-Terminal Workflows (tmux) ==="
    echo ""
    echo "Sessions:"
    echo "  h_dev_session      - Full development session with 4 windows:"
    echo "                         0: Main (for launching system)"
    echo "                         1: Monitor (controller status)"
    echo "                         2: Logs (error checking)"
    echo "                         3: Development (coding)"
    echo ""
    echo "  h_monitor_session  - Dedicated monitoring session with 3 panes:"
    echo "                         - Controller status (top-left)"
    echo "                         - Joint states (right)"
    echo "                         - Resource usage (bottom-left)"
    echo ""
    echo "  h_kill_sessions    - Kill all arm tmux sessions"
    echo ""
    echo "tmux Navigation:"
    echo "  Ctrl+b then number   - Switch between windows"
    echo "  Ctrl+b then arrow    - Switch between panes"
    echo "  Ctrl+b then d        - Detach from session"
    echo "  tmux attach -t humanoid_dev - Re-attach to session"
    echo ""
    echo "Note: tmux must be installed (sudo apt install tmux)"
}

h_help_codegen() {
    echo "=== Code Generation Templates ==="
    echo ""
    echo "Create New Files:"
    echo "  h_new_node <package> <node_name>"
    echo "    - Creates ROS 2 Python node with boilerplate"
    echo "    - Includes publisher, subscriber, timer, parameters"
    echo "    - Automatically executable with correct shebang"
    echo ""
    echo "  h_new_launch <package> <launch_name>"
    echo "    - Creates Python launch file template"
    echo "    - Includes launch arguments and example node"
    echo ""
    echo "Examples:"
    echo "  h_new_node humanoid_control trajectory_optimizer"
    echo "  h_new_launch humanoid_control custom_system"
    echo ""
    echo "Note: Remember to add new nodes to package setup.py!"
}

h_help_docs() {
    echo "=== Documentation Helpers ==="
    echo ""
    echo "View Documentation:"
    echo "  h_docs             - Navigate to docs folder"
    echo "  h_readme           - View CLAUDE.md (project guide)"
    echo ""
    echo "Package Information:"
    echo "  h_pkg_info <pkg>   - Show package prefix and executables"
    echo "  h_interfaces <pkg> - List messages/services/actions in package"
    echo "  h_interface_show <interface> - Show interface definition"
    echo ""
    echo "Examples:"
    echo "  h_pkg_info humanoid_control"
    echo "  h_interfaces sensor_msgs"
    echo "  h_interface_show sensor_msgs/msg/JointState"
}

h_help_deps() {
    echo "=== Dependency Management ==="
    echo ""
    echo "Install & Check:"
    echo "  h_deps_install     - Install all missing dependencies"
    echo "  h_deps_check       - Check for missing dependencies"
    echo "  h_deps_update      - Update rosdep database"
    echo ""
    echo "Package Dependencies:"
    echo "  h_deps_list <pkg>  - List dependencies for specific package"
    echo ""
    echo "Examples:"
    echo "  h_deps_check       # Check what's missing"
    echo "  h_deps_install     # Install missing deps"
    echo "  h_deps_list humanoid_control  # Show humanoid_control dependencies"
}

h_help_backup() {
    echo "=== Backup & Restore ==="
    echo ""
    echo "Configuration Backup:"
    echo "  h_backup_config    - Backup all YAML, RVIZ, SRDF config files"
    echo "  h_list_backups     - List all available backups"
    echo ""
    echo "Backups include:"
    echo "  - All .yaml configuration files"
    echo "  - All .rviz visualization configs"
    echo "  - All .srdf MoveIt configs"
    echo ""
    echo "Backups are saved to: $PROJECT_ROOT/backups/"
    echo "Format: backups/config_YYYYMMDD_HHMMSS/"
}

# ============================================================================
# Initialization
# ============================================================================

# Auto-source workspace if in project directory
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
    echo "✅ Workspace sourced from $PROJECT_ROOT"
else
    echo "⚠️  Workspace not built yet. Run 'cb' to build."
fi

# Show help on first load
echo ""
echo "🤖 LD Robots Humanoid Robot Demo shortcuts loaded!"
echo "   Type 'h_help' to see all available commands."
echo "   Type 'h_home' to navigate to project root."
echo "   Type 'h_rviz' to visualize the robot."
echo ""
