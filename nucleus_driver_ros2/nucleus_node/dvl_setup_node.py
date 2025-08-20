import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from interfaces.srv import ConnectTcp, Command, Start

class DVLSetupNode(Node):
    def __init__(self):
        super().__init__('dvl_setup_node')
        # --- Decalre Parameters ---
        self.declare_parameter('use_prev_config', False)
        self.use_prev_config = self.get_parameter('use_prev_config').get_parameter_value().bool_value

        # --- Create service clients ---
        self.connect_client = self.create_client(ConnectTcp, '/nucleus_node/connect_tcp')
        self.getimu_client = self.create_client(Command, '/nucleus_node/command')
        self.setimu_client = self.create_client(Command, '/nucleus_node/command')
        self.setahrs_client = self.create_client(Command, '/nucleus_node/command')
        self.settrig_client = self.create_client(Command, '/nucleus_node/command')
        self.start_mes_client = self.create_client(Start, '/nucleus_node/start')
        # --- Wait for services to be available ---
        for client in [self.connect_client, self.getimu_client, self.start_mes_client]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {client.srv_name} not available yet. Waiting...')

        # --- Perform setup ---
        self.setup_dvl()

    def setup_dvl(self):
        self.get_logger().info("Starting DVL setup...")

        # 1. Connect TCP
        self.get_logger().info("Connecting to DVL via TCP...")
        tcp_req = ConnectTcp.Request()
        tcp_req.host = "192.168.2.11"
        tcp_req.password = "nortek"   
        tcp_resp = self.call_service(self.connect_client, tcp_req)
        self.get_logger().info(f"TCP connection response: {tcp_resp.status}")
        if(self.use_prev_config):
            self.get_logger().info(f"Using previous configuration")
        else:
            # 2. Check IMU and DS
            self.get_logger().info("Checking IMU status...")
            imu_req = Command.Request()
            imu_req.command = 'GETIMU'
            imu_req.nmea = False
            imu_resp = self.call_service(self.getimu_client, imu_req)
            self.get_logger().info(f"IMU Configuration: {imu_resp.reply}")
            imu_resp_split = imu_resp.reply.split(",")
            imu_DS = imu_resp_split[1].strip('"')
            #print(imu_DS)
            
            # 3. Switch on DS if not ON
            if(imu_DS!= 'ON'):
                self.get_logger().info("Switching ON IMU DataStream...")
                set_imu_req = Command.Request()
                set_imu_req.command = 'SETIMU,DS="ON"'
                set_imu_req.nmea = False
                set_imu_resp = self.call_service(self.setimu_client, set_imu_req)
                self.get_logger().info(f"SETIMU command response: {set_imu_resp.reply}")

            # 4. Switch on AHRS DS
            self.get_logger().info("Switching ON AHRS DataStream...")
            set_ahrs_req = Command.Request()
            set_ahrs_req.command = 'SETAHRS,DS="ON"'
            set_ahrs_req.nmea = False
            set_ahrs_resp = self.call_service(self.setahrs_client, set_ahrs_req)
            self.get_logger().info(f"SETAHRS command response: {set_ahrs_resp.reply}")

            # 5. Setting up Trigger for Bottomtracking
            self.get_logger().info("Setting up Trigger to get Bottom Track...")
            set_trig_req = Command.Request()
            set_trig_req.command = 'SETTRIG,SRC="INTERNAL",ALTI=2,CP=0'
            set_trig_req.nmea = False
            set_trig_resp = self.call_service(self.settrig_client, set_trig_req)
            self.get_logger().info(f"SETTRIG command response: {set_trig_resp.reply}")
        
        #4. Start taking measurement
        self.get_logger().info("Sending START command...")
        start_req = Start.Request()
        start_resp = self.call_service(self.start_mes_client, start_req)
        self.get_logger().info(f"START command response: {start_resp.reply}")
        self.get_logger().info("DVL setup complete and started taking measurement!")

    def call_service(self, client, request):
        """Helper to call a service synchronously using a temporary executor"""
        future = client.call_async(request)
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future)
        executor.shutdown()
        return future.result()


def main():
    rclpy.init()
    node = DVLSetupNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
