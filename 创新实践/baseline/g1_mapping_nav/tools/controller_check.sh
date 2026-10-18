# Source from a simulator validation script; never infer activation from publishers.
controller_state() {
    timeout 20 ros2 service call /robot1/controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers 2>/dev/null \
        | grep -oE "name='$1', state='[a-z]+'" \
        | sed -E "s/.*state='([a-z]+)'.*/\1/"
}

require_active_controllers() {
    local logs="$1" name state
    for name in joint_group_controller joint_state_broadcaster; do
        state="$(controller_state "${name}")"
        if [[ -z "${state}" ]]; then
            echo "ERROR: 无法确认 ${name} 状态；请用加载 gz_ros2_control 的完整仿真启动入口。" >&2
            return 1
        fi
        if [[ "${state}" == unconfigured ]]; then
            timeout 30 ros2 service call /robot1/controller_manager/configure_controller \
                controller_manager_msgs/srv/ConfigureController "{name: ${name}}" \
                >"${logs}/configure_${name}.log" 2>&1 || return 1
            state="$(controller_state "${name}")"
        fi
        if [[ "${state}" == inactive ]]; then
            timeout 30 ros2 service call /robot1/controller_manager/switch_controller \
                controller_manager_msgs/srv/SwitchController \
                "{activate_controllers: [${name}], strictness: 2}" \
                >"${logs}/activate_${name}.log" 2>&1 || return 1
            state="$(controller_state "${name}")"
        fi
        if [[ "${state}" != active ]]; then
            echo "ERROR: ${name} = ${state:-查不到}，停止验收。" >&2
            return 1
        fi
        echo "   ${name}: active"
    done
}
