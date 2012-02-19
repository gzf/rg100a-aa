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

enable_broadcom() {
    local device="$1"
    local channel country macfilter maclist txpower

    config_get channel $device channel
    config_get country $device country
    config_get macfilter $device macfilter
    config_get txpower $device txpower

    kill_nas

    if [ -n "$country" ]; then
        $WLCTL -i $device country $country
    fi

    $WLCTL -i $device channel ${channel:-11}

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

    config_get vifs $device vifs
    for vif in $vifs; do
        setup_iface $vif
    done

    $WLCTL -i $device up

    for vif in $vifs; do
        ifup_iface $vif
    done
}

genwepkey() {
    for keylen in 5 13 16; do
        if [ ${#1} -eq $keylen ]; then
            echo -n "$1" | hexdump -e "$keylen/1 \"%02x\""
            return
        fi
    done
    echo -n $1
}

setup_iface() {
    local vif="$1"
    local ifname mode ssid

	config_get ifname "$vif" device
    config_get mode "$vif" mode
    case $mode in
        ap)
            local isolate
            config_get isolate "$vif" isolate
            $WLCTL -i $ifname ap 1
            $WLCTL -i $ifname ap_isolate ${isolate:-0}
            ;;
        *)
            echo "wireless mode \`$mode\' not supported yet."
            return 1
            ;;
    esac

    config_get ssid "$vif" ssid
    $WLCTL -i $ifname ssid $ssid

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
				    $WLCTL -i $ifname addwep $((index - 1)) `genwepkey $key`
                done
            else
                if [ "$key" -ge 1 ] &> /dev/null && [ "$key" -le 4 ] &> /dev/null; then
	                config_get key "$vif" key$key
                fi
			    $WLCTL -i $ifname addwep 0 `genwepkey $key`
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
    local vif="$1"
	config_get ifname "$vif" device

	local net_cfg
	net_cfg="$(find_net_config "$vif")"
	[ -z "$net_cfg" ] || {
	    set_wifi_up $vif $ifname
		start_net $ifname $net_cfg `$WLCTL -i $ifname bssid`
	}
}

detect_broadcom() {
	local i=-1

	while [ -f /proc/net/wl$((++i)) ]; do
		config_get type wl${i} type
		[ "$type" = broadcom ] && continue
		cat <<EOF
config wifi-device  wl${i}
    option type     broadcom
    option channel  11

    # REMOVE THIS LINE TO ENABLE WIFI:
    option disabled 1

config wifi-iface
    option device      wl${i}
    option network     lan
    option mode        ap
    option ssid        OpenWrt${i}
    option encryption  psk2
    option key         secret-key
EOF
	done
}
