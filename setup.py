entry_points={
    'console_scripts': [
        # Remove or comment out:
        # 'imu_publisher = imu_sim.imu_publisher:main',

        
        'imu_serial_node = imu_sim.imu_serial_node:main',
        'kalman_node     = imu_sim.kalman_node:main',    
    ],
},
