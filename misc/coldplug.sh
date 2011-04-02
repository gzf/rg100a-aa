coldplug_interface_pppoe() {
    local config="$1"
    config_get ifname "$config" ifname

    [ -n "$ifname" ] && grep "^$ifname:" /proc/net/dev &> /dev/null && \
        setup_interface "$ifname" "$config"
}
