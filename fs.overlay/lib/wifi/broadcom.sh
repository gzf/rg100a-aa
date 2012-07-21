# Author: Zhifeng Gu <guzhifeng1979@hotmail.com>
# Date: 2011/03
# License: GPL

append DRIVERS "broadcom"

WLCTL=/bin/wl

scan_broadcom() {
	local device="$1"
}

kill_nas() {
    for pid in `find /var/run -name "nas.*.pid"`; do
        start-stop-daemon -K -s KILL -p $pid 1>&- 2>&-
        rm -f $pid
    done
}

disable_broadcom() {
    local device="$1"
    set_wifi_down "$device"
    kill_nas
    for index in  0 1 2 3; do
        $WLCTL -i "$device" bss -C $index down
    done
    $WLCTL -i "$device" down
    (
        include /lib/network

        # make sure the interfaces are down and removed from all bridges
        for dev in $device ${device}.1 ${device}.2 ${device}.3; do
            ifconfig "$dev" down 2>/dev/null >/dev/null && {
                unbridge "$dev"
            }
        done
    )
    true
}

gen_ifname() {
    local device=$1 index=$2
    if [ $index -gt 0 ]; then
        echo $device.$index
    else
        echo $device
    fi
}

enable_broadcom() {
    local device="$1"
    local channel country macfilter maclist txpower macaddr

    kill_nas

    $WLCTL -i $device phy_watchdog 0
    $WLCTL -i $device mbss 1
    $WLCTL -i $device ap 1
    
    config_get channel $device channel
    config_get country $device country
    config_get macfilter $device macfilter
    config_get txpower $device txpower
    config_get macaddr $device macaddr

    $WLCTL -i $device channel ${channel:-11}
    if [ -n "$country" ]; then
        $WLCTL -i $device country $country
    fi
    if [ -n "$macaddr" ]; then
        $WLCTL -i $device bssid $macaddr
    else
        $WLCTL -i $device bssid `$WLCTL -i $device cur_etheraddr`
    fi
    case $macfilter in
        deny)
            $WLCTL -i $device macmode 1 ;;
        allow)
            $WLCTL -i $device macmode 2 ;;
        *)
            $WLCTL -i $device macmode 0 ;;
    esac
    if [ -n "$macfilter" ] && ( [ "$macfilter" = deny ] || [ "$macfilter" = allow ] ); then
        config_get maclist $device maclist
        $WLCTL -i $device mac $maclist
    fi

    if [ -n "$txpower" ]; then
        $WLCTL -i $device txpwr1 -o -d $txpower
    fi

    local vifs ifname count ssid ifupcmd
    config_get vifs $device vifs
    for vif in $vifs; do
        count=`eval echo \\${${device}_COUNT}`
        if [ -z "$count" ]; then
            eval local ${device}_COUNT=0
            count=0
        fi

        config_get ssid "$vif" ssid
        $WLCTL -i $device ssid -C $count $ssid

        ifname=$(gen_ifname $device $count)
        setup_iface $vif $ifname
        if [ $? -eq 0 ]; then
            $WLCTL -i $device bss -C $count up
            ifupcmd="$ifupcmd ifup_iface $vif $ifname;"
        fi

        eval ${device}_COUNT=\$\(\(${device}_COUNT + 1\)\)
    done

    $WLCTL -i $device phy_watchdog 1
    
    if [ -n "$ifupcmd" ]; then
        eval $ifupcmd
    fi
}

setup_iface() {
    local vif="$1" ifname="$2"
    local mode ssid bssid isolate

    config_get mode "$vif" mode "ap"
    case $mode in
        ap)
            config_get isolate "$vif" isolate "0"
            $WLCTL -i $ifname ap_isolate $isolate
            ;;
        wds)
            config_get bssid "$vif" bssid
            $WLCTL -i $ifname wds $bssid
            return
            ;;
        *)
            echo "wireless mode \`$mode\' not supported yet."
            return 1
            ;;
    esac

    #
    # Next: deal with security
    #
    local wsec=0 auth=0 wpa=0 ciphers=0 eap=0 enc key nasopts
	config_get enc "$vif" encryption
	config_get key "$vif" key

    case "$enc" in
        psk2*)
            wpa=128; ciphers=4;;
        wpa2*)
            wpa=64;  ciphers=4;;
        psk*)
            wpa=4;   ciphers=2;;
        wpa*)
            wpa=2;   ciphers=2;;
        mixed-psk*)
            wpa=132; ciphers=6;;
        mixed-wpa*)
            wpa=66;  ciphers=6;;
        wep*)
            wsec=1
            if [ "$enc" = wep+shared ]; then
                auth=1
            fi
            if [ -z "$key" ]; then
                for index in 1 2 3 4; do
				    config_get key "$vif" key$index
				    [ -n "$key" ] || continue
				    $WLCTL -i $ifname addwep $((index - 1)) $key
                done
            else
                if [ "$key" -ge 1 ] &> /dev/null && [ "$key" -le 4 ] &> /dev/null; then
	                config_get key "$vif" key$key
                fi
			    $WLCTL -i $ifname addwep 0 $key
            fi
            ;;
        none)
            ;;
        *)
            echo "unknown encryption method: $enc"
            return 2
    esac

    case "$enc" in
        *+tkip*) wsec=$(($wsec + 2))
    esac
    case "$enc" in
        *+ccmp*|*+aes*) wsec=$(($wsec + 4))
    esac
    # if no +tkip or +ccmp specified, then using the default ciphers
    if [ $wpa -ne 0 ] && [ $wsec -eq 0 ]; then
        wsec=$ciphers
    fi

    case "$enc" in
        *psk*)
            nasopts="-k $key";;
        *wpa*)
            eap=1
            local server port
	        config_get server "$vif" server
	        config_get port "$vif" port
	        nasopts="-r $key -h $server -p ${port:-1812}"
    esac

    $WLCTL -i $ifname wsec $wsec
    $WLCTL -i $ifname auth $auth
    $WLCTL -i $ifname wpa_auth $wpa
    if [ $wsec -ne 0 ]; then
        $WLCTL -i $ifname wsec_restrict 1
    else
        $WLCTL -i $ifname wsec_restrict 0
    fi
    $WLCTL -i $ifname eap $eap

	[ -z "$nasopts" ] || {
	    local netcfg bridge
        netcfg="$(find_net_config "$vif")"
	    bridge="$(bridge_interface "$netcfg")"

		nas -P /var/run/nas.$ssid.pid ${bridge:+ -l $bridge} -i $ifname \
            -A -m $wpa -w $wsec -s $ssid -g 3600 $nasopts &
	}
}

ifup_iface() {
    local vif=$1 ifname=$2
	local net_cfg="$(find_net_config $vif)"

	[ -z "$net_cfg" ] || {
	    set_wifi_up $vif $ifname
		start_net $ifname $net_cfg `$WLCTL -i $ifname bssid`
	}
}

detect_broadcom() {
    local i=-1

    while [ -f /proc/net/wl$((++i)) ]; do
        cat <<EOF
config wifi-device  wl${i}
    option type     broadcom
    option channel  11
    #option country CN
    #option txpower 20
    #option macaddr   12:34:56:78:9A:BC
    #option macfilter allow
    #option maclist   '12:34:56:78:90:ED 35:46:80:E5:06:33'
    # REMOVE THIS LINE TO ENABLE WIFI:
    option disabled 1

config wifi-iface
    option device      wl${i}
    option network     lan
    option mode        ap
    option ssid        OpenWrt${i}
    option encryption  psk2
    option key         secret-key
    #option isolate    1

#config wifi-iface
#    option device      wl${i}
#    option network     lan
#    option mode        ap
#    option ssid        WEPdemo${i}
#    option encryption  wep+shared
#    option key         short
EOF
    done
}
