from tm_msgs.msg import *
from tm_msgs.srv import *
from custom_msg.srv import *
import rospy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class TM:
    '''
    這裡是放手臂會用到的Function
    '''

    def __init__(self) -> None:
        self.tool_x = self.tool_y = self.tool_z = self.tool_rx = self.tool_ry = self.tool_rz = None

    def feedback_callback(self, msg: FeedbackState): # 手臂姿態
        self.tool_x = msg.tool_pose[0]*1000
        self.tool_y = msg.tool_pose[1]*1000
        self.tool_z = msg.tool_pose[2]*1000
        self.tool_rx = msg.tool_pose[3]*180/3.1415926
        self.tool_ry = msg.tool_pose[4]*180/3.1415926
        self.tool_rz = msg.tool_pose[5]*180/3.1415926
        # arm_pose = (ax, ay, az, arx, ary, arz)
        # tool_pose = (msg.tool_pose[0], msg.tool_pose[1], msg.tool_pose[2], msg.tool_pose[3], msg.tool_pose[4], msg.tool_pose[5])
        # pub_tf_static(tool_pose[0], tool_pose[1], tool_pose[2], tool_pose[3], tool_pose[4], tool_pose[5], "arm", "tool")

    def send_script_client(self, cmd: str):
        rospy.wait_for_service('tm_driver/send_script')
        try:
            tm_send_script = rospy.ServiceProxy('tm_driver/send_script', SendScript)
            resp1 = tm_send_script("demo", cmd)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def send_gripper_client(self, cmd: bool): # True: 夾取, False: 放開
        '''
        SetIORequest_()
        : module(0)
        , type(0)
        , pin(0)
        , state(0.0)  {
        }
        '''
        rospy.wait_for_service('tm_driver/set_io')
        try:
            tm_send_io = rospy.ServiceProxy('tm_driver/set_io', SetIO)
            if cmd == True:
                resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_ON)
            elif cmd == False:
                resp1 = tm_send_io(SetIORequest.MODULE_ENDEFFECTOR, SetIORequest.TYPE_DIGITAL_OUT, 0, SetIORequest.STATE_OFF)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

class Tranform:
    '''
    這裡是放歐拉角和四元數的Function
    '''

    def euler_to_orientation(self, roll, pitch, yaw):
        '''
        Convert euler angles to quaternion \n
        euler: (roll, pitch, yaw) in degrees \n
        quaternion: (x, y, z, w)
        '''
        # Convert degrees to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Calculate quaternion components
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        
        # Normalize quaternion
        length = math.sqrt(w**2 + x**2 + y**2 + z**2)
        w /= length
        x /= length
        y /= length
        z /= length
        
        return np.array([x, y, z, w])

    def orientation_to_euler(self, x, y, z, w):
        '''
        Convert quaternion to euler angles \n
        euler: (roll, pitch, yaw) in degrees \n
        quaternion: (x, y, z, w)
        '''
        # Convert quaternion components to euler angles in radians
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        # Convert to degrees
        roll_x = math.degrees(roll_x)
        pitch_y = math.degrees(pitch_y)
        yaw_z = math.degrees(yaw_z)
        
        return np.array([roll_x, pitch_y, yaw_z])

class TF(Tranform):
    '''
    這裡是放ROS TF會用到的Function
    '''

    def pub_tf(self, tf, header_frame_id="arm", child_frame_id="tool"):        
        '''
            Publish tf with euler angles \n
            - tf: (x, y, z, roll, pitch, yaw) in mm and degrees
            - header_frame_id: parent frame id
            - child_frame_id: child frame id
        '''
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = tf[0]
        t.transform.translation.y = tf[1]
        t.transform.translation.z = tf[2]

        quat = self.euler_to_orientation(tf[3], tf[4], tf[5])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        br.sendTransform(t)

    def pub_static_tf(self, tf, header_frame_id="tool", child_frame_id="tool_target"):
        '''
            Publish static tf with euler angles \n
            - tf: (x, y, z, roll, pitch, yaw) in mm and degrees
            - header_frame_id: parent frame id
            - child_frame_id: child frame id
        '''
        br = StaticTransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = tf[0]
        t.transform.translation.y = tf[1]
        t.transform.translation.z = tf[2]

        quat = self.euler_to_orientation(tf[3], tf[4], tf[5])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        br.sendTransform(t)

    def pub_static_tf_orientation(self, tf, header_frame_id, child_frame_id):
        '''
            Publish static tf with orientation \n
            - tf: (x, y, z, qx, qy, qz, qw) in mm and quaternion
            - header_frame_id: parent frame id
            - child_frame_id: child frame id
        '''
        br = StaticTransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = tf[0]
        t.transform.translation.y = tf[1]
        t.transform.translation.z = tf[2]

        t.transform.rotation.x = tf[3]
        t.transform.rotation.y = tf[4]
        t.transform.rotation.z = tf[5]
        t.transform.rotation.w = tf[6]
        br.sendTransform(t)


    def get_tf(self, header_frame, child_frame, m2mm = False): # 取得tf,  return transform
        '''
        Get the transform between two frames \n
            - header_frame: parent frame id
            - child_frame: child frame id
            -----------------------------------
            return transform (translation, rotation)
        '''
        tfBuffer = Buffer()
        _listener = TransformListener(tfBuffer)
        while not rospy.is_shutdown():
            try:
                t = tfBuffer.lookup_transform(header_frame, child_frame, rospy.Time())
                transform = t.transform
                # rospy.loginfo(transform)
                
                if m2mm:
                    transform.translation.x = transform.translation.x * 1000
                    transform.translation.y = transform.translation.y * 1000
                    transform.translation.z = transform.translation.z * 1000
                
                return transform
            except Exception as e:
                # print(e)
                pass

def req_action(obs):
    rospy.wait_for_service('/get_action')
    get_action = rospy.ServiceProxy('/get_action', rl)
    req = rlRequest()
    req.cubeA_pos = (0, 0, 0)
    req.cubeA_quat = (0, 0, 0, 1)
    req.cubeB_pos = (0, 0, 0)
    req.cubeB_quat = (0, 0, 0, 1)
    req.cubeC_pos = (0, 0, 0)
    req.cubeC_quat = (0, 0, 0, 1)
    req.eef_pos = (0, 0, 0)
    req.eef_quat = (0, 0, 0, 1)
    req.obj = 0
    req.task = 0
    req.target_pos = (0, 0, 0)
    req.target_quat = (0, 0, 0, 1)
    req.q_gripper = (0, 0)
    
    action = get_action(req).action
    arm = action[:6]
    gripper = action[6]
    rospy.loginfo(f'arm: {arm}')
    rospy.loginfo(f'gripper: {gripper}')
    return action