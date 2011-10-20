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
    local channel country macfilter maclist

    config_get channel $device channel
    config_get country $device country
    config_get macfilter $device macfilter

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
    
    config_get vifs $device vifs
    for vif in $vifs; do
        setup_iface $vif
    done

    $WLCTL -i $device up

    for vif in $vifs; do
        ifup_iface $vif
    done
}

setup_iface() {    
    local vif="$1"

	local wsec_r=0
	local eap_r=0
	local wsec=0
	local auth=0
	local nasopts=

	config_get ifname "$vif" device
	config_get enc "$vif" encryption

	case "$enc" in
		*WEP*|*wep*)
			wsec_r=1
			wsec=1
			defkey=1
			config_get key "$vif" key
			case "$enc" in
				*shared*) $WLCTL -i $ifname wepauth 1;;
				*) $WLCTL -i $ifname wepauth 0;;
			esac
			case "$key" in
				[1234])
					for knr in 1 2 3 4; do
						config_get k "$vif" key$knr
						[ -n "$k" ] || continue
						$WLCTL -i $ifname addwep $knr $k
					done
					;;
				"");;
				*) $WLCTL -i $ifname addwep 1 $key;;
			esac
			;;
		*psk*|*PSK*)
			wsec_r=1
			config_get key "$vif" key
			case "$enc" in
				wpa*+wpa2*|WPA*+WPA2*|*psk+*psk2|*PSK+*PSK2) auth=132; wsec=6;;
				wpa2*|WPA2*|*PSK2|*psk2) auth=128; wsec=4;;
				*aes|*AES) auth=4; wsec=4;;
				*) auth=4; wsec=2;;
			esac
			nasopts="-k $key"
			;;
		*wpa*|*WPA*)
			wsec_r=1
			eap_r=1
			config_get key "$vif" key
			config_get server "$vif" server
			config_get port "$vif" port
			case "$enc" in
				wpa*+wpa2*|WPA*+WPA2*) auth=66; wsec=6;;
				wpa2*|WPA2*) auth=64; wsec=4;;
				*) auth=2; wsec=2;;
			esac
			nasopts="-r $key -h $server -p ${port:-1812}"
			;;
	esac
    $WLCTL -i $ifname wsec $wsec
    $WLCTL -i $ifname wpa_auth $auth
    $WLCTL -i $ifname wsec_restrict $wsec_r
    $WLCTL -i $ifname eap_restrict $eap_r

    config_get ssid "$vif" ssid
    config_get mode "$vif" mode

    $WLCTL -i $ifname ssid $ssid
    [ $mode = ap ] && {
        local isolate
        config_get isolate "$vif" isolate
        $WLCTL -i $ifname ap 1
        $WLCTL -i $ifname ap_isolate ${isolate:-0}
    }
	[ $mode = monitor ] && {
		$WLCTL -i $ifname monitor $monitor
		$WLCTL -i $ifname passive $passive
	}

	[ -z "$nasopts" ] || {
		local nas_mode="-A"
		[ $mode = sta ] && nas_mode="-S"

	    local netcfg bridge
        netcfg="$(find_net_config "$vif")"
	    bridge="$(bridge_interface "$netcfg")"

		nas -P /var/run/nas.$ssid.pid ${bridge:+ -l $bridge} -i $ifname \
            $nas_mode -m $auth -w $wsec -s $ssid -g 3600 $nasopts &
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
    option network	   lan
    option mode        ap
    option ssid        OpenWrt${i#0}
    option encryption  psk2
    option key         <secret-key>
EOF
	done
}
