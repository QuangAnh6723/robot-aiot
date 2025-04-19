#!/bin/bash

# Kiểm tra tham số đầu vào
if [ "$#" -ne 2 ]; then
    echo ": $0 <SSID> <PASSWORD>"
    exit 1
fi

SSID="$1"
PASSWORD="$2"
NETPLAN_FILE="/etc/netplan/50-cloud-init.yaml"

# Ghi cấu hình WiFi vào file Netplan
echo "cap nhat wifi"
sudo bash -c "cat > $NETPLAN_FILE <<EOF
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        \"$SSID\":
          password: \"$PASSWORD\"
EOF"

# # Áp dụng thay đổi
# echo "config Netplan..."
# sudo netplan apply

# # Đợi vài giây để WiFi kết nối
# sleep 5

# # Kiểm tra trạng thái WiFi
# echo "doi ket noi wifi"
# if iwconfig wlan0 | grep -q "ESSID:\"$SSID\""; then
#     echo "connect WiFi: $SSID"
#     exit 0
# else
#     echo "can not connect WiFi: $SSID"
#     exit 1
# fi
