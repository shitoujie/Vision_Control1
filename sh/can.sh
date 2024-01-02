echo "nuc" | sudo -S ifconfig eno1 192.168.0.105

echo "nuc" | sudo -S ip link set can0 type can bitrate 1000000
echo "nuc" | sudo -S ifconfig can0 up
echo "nuc" | sudo -S ip link set can1 type can bitrate 1000000
echo "nuc" | sudo -S ifconfig can1 up
