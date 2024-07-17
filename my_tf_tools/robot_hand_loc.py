import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import tf2_ros
import geometry_msgs.msg
import json
import numpy as np
from threading import Thread
from scipy.spatial.transform import Rotation as R

# MQTT ブローカーの情報
broker_address = "192.168.207.22"
broker_port = 1883
topic = "new_robot_pose"

class Robot_hand(Node):

    def __init__(self):
        super().__init__('Robot_hand')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 1.0
        self.y = 0.0
        self.z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0

        # MQTT クライアントのセットアップ
        self.client = mqtt.Client()
        # MQTT ブローカーに接続
        self.client.connect(broker_address, broker_port, 60)
        # メッセージ受信時のコールバック関数を設定
        self.client.on_message = self.on_message
        # MQTT クライアントが指定したトピックをサブスクライブ
        self.client.subscribe(topic)
        # MQTT クライアントのループをスレッドで実行
        self.mqtt_thread = Thread(target=self.client.loop_forever)
        self.mqtt_thread.start()

    def on_message(self, client, userdata, message):
        print("Message Received")
        # JSON形式のメッセージをデコード
        nmsg_dict = json.loads(message.payload.decode("utf-8"))
        self.x = float(nmsg_dict['x']) / 100
        self.y = float(nmsg_dict['y']) / 100
        self.z = float(nmsg_dict['z']) / 100
        self.rot_x = float(nmsg_dict['rot_x']) * np.pi/180
        self.rot_y = float(nmsg_dict['rot_y']) * np.pi/180
        self.rot_z = float(nmsg_dict['rot_z']) * np.pi/180
        
        t1 = geometry_msgs.msg.TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'end_effector'
        
        t1.transform.translation.x = self.x
        t1.transform.translation.y = self.y
        t1.transform.translation.z = self.z
        rot1 = R.from_euler('xyz', [self.rot_x, self.rot_y, self.rot_z])
        rot0 = R.from_euler('xyz', [np.pi/2, 0.0, np.pi/2])
        rot2 = rot0 * rot1
        q = rot2.as_quat()
        #q = self.euler_to_quaternion(self.rot_x, self.rot_y, self.rot_z)
        t1.transform.rotation.x = q[0]
        t1.transform.rotation.y = q[1]
        t1.transform.rotation.z = q[2]
        t1.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t1)
        

        # end_effectorからcamera_linkへのTransform
        t2 = geometry_msgs.msg.TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'end_effector'
        t2.child_frame_id = 'camera_link'
        
        t2.transform.translation.x = 0.0  # x座標
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0  # エンドエフェクタからカメラまでの距離
        
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)
        

    def euler_to_quaternion(self, roll, pitch, yaw):
        # オイラー角をクォータニオンに変換
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    hand = Robot_hand()
    try:
        rclpy.spin(hand)
    except KeyboardInterrupt:
        pass
    hand.client.disconnect()
    hand.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
