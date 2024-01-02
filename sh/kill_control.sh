
hw_pid=$(ps -ef | grep "gimbal_hw_node" | grep -v grep | awk '{print $2}')
chassis_pid=$(ps -ef | grep "chassis_demo_no" | grep -v grep | awk '{print $2}')
gimbal_pid=$(ps -ef | grep "gimbal_demo_nod" | grep -v grep | awk '{print $2}')
remote_pid=$(ps -ef | grep "remote_sensor_n" | grep -v grep | awk '{print $2}')
imu_pid=$(ps -ef | grep "imu_sensor_node" | grep -v grep | awk '{print $2}')

echo "nuc" | sudo -S kill -9 $hw_pid
echo "nuc" | sudo -S kill -9 $chassis_pid
echo "nuc" | sudo -S kill -9 $gimbal_pid
echo "nuc" | sudo -S kill -9 $remote_pid
echo "nuc" | sudo -S kill -9 $imu_pid
