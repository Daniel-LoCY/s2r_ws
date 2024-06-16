from tm_msgs.msg import *
from tm_msgs.srv import *
from custom_msg.srv import *
import rospy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

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

class TF():
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

        quat = quaternion_from_euler(tf[3], tf[4], tf[5])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        br.sendTransform(t)

    def pub_tf_orientation(self, tf, header_frame_id="arm", child_frame_id="tool"):        
        '''
            Publish tf with euler angles \n
            - tf: (x, y, z, qx, qy, qz, qw) in mm and quaternion
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

        t.transform.rotation.x = tf[3]
        t.transform.rotation.y = tf[4]
        t.transform.rotation.z = tf[5]
        t.transform.rotation.w = tf[6]
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

        quat = quaternion_from_euler(tf[3], tf[4], tf[5])
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