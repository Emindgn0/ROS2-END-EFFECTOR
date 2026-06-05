#!/usr/bin/env python3
# KALDIRILDI: Bu node gazebo_bridge.py ile birleştirildi.
# IK artık gazebo_bridge içinde çalışıyor (/cartesian_interface/arm/reference subscriber'ı orada).
# Bu dosya geriye dönük uyumluluk için bırakıldı, launch'ta kullanılmıyor.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String
import math
import numpy as np

JOINTS_DATA = [
    ([0.0,    0.0,     0.3443], [0.0,        0.0,        0.0      ]),
    ([0.0,    0.0099,  0.0   ], [0.0,       -math.pi/2, -math.pi/2]),
    ([0.7595, 0.0,     0.0   ], [0.0,        0.0,        math.pi/2]),
    ([0.0,   -0.6195,  0.0   ], [math.pi/2,  0.0,        0.0      ]),
    ([0.0,    0.0,     0.0   ], [-math.pi/2, 0.0,        0.0      ]),
    ([0.0,   -0.121,   0.0   ], [math.pi/2,  0.0,        0.0      ]),
]
TOOL_XYZ = [-0.001, 0.0, 0.12]
TOOL_RPY = [0.0, math.pi, math.pi]
HOME_JOINTS = [0.0, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]

def rot_x(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]],dtype=float)
def rot_y(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]],dtype=float)
def rot_z(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
def trans(x,y,z):
    T=np.eye(4); T[0,3]=x; T[1,3]=y; T[2,3]=z; return T
def joint_transform(xyz,rpy,theta):
    x,y,z=xyz; r,p,yw=rpy
    return trans(x,y,z)@rot_x(r)@rot_y(p)@rot_z(yw)@rot_z(theta)

_TOOL_T = joint_transform(TOOL_XYZ, TOOL_RPY, 0.0)

def forward_kinematics(joints):
    T=np.eye(4)
    for i,(xyz,rpy) in enumerate(JOINTS_DATA):
        T=T@joint_transform(xyz,rpy,joints[i])
    return T @ _TOOL_T

def inverse_kinematics(target_pos, q_init=None, max_iter=1000, alpha=0.3):
    if q_init is None: q_init=HOME_JOINTS.copy()
    q=np.array(q_init,dtype=float)
    for _ in range(max_iter):
        T=forward_kinematics(q); pos=T[:3,3]
        err=np.array(target_pos)-pos
        if np.linalg.norm(err)<1e-4: break
        J=np.zeros((3,6)); delta=1e-6
        for i in range(6):
            qd=q.copy(); qd[i]+=delta
            J[:,i]=(forward_kinematics(qd)[:3,3]-pos)/delta
        q+=alpha*np.linalg.pinv(J)@err
        q=np.clip(q,-2*math.pi,2*math.pi)
    T=forward_kinematics(q)
    return q.tolist(), float(np.linalg.norm(np.array(target_pos)-T[:3,3]))

class CartesianToJointNode(Node):
    def __init__(self):
        super().__init__('cartesian_to_joint_node')
        self.get_logger().warn(
            'cartesian_to_joint_node artık kullanılmıyor — IK gazebo_bridge içinde çalışıyor!'
        )
        self._current_joints = HOME_JOINTS.copy()
        self._servo_pos = 0.0
        self.create_subscription(PoseStamped,'/cartesian_interface/arm/reference',self._cb_pose,10)
        self.pub_joints = self.create_publisher(Float64MultiArray,'/gz/dsr_position_controller/commands',10)
        self.pub_log = self.create_publisher(String,'/end_effector/log',10)

    def _cb_pose(self, msg):
        target=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        joints, err = inverse_kinematics(target, self._current_joints)
        if err > 0.05:
            self.get_logger().warn(f'IK hata={err*1000:.1f}mm')
            return
        self._current_joints = joints
        cmd = Float64MultiArray()
        cmd.data = joints + [self._servo_pos]
        self.pub_joints.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node=CartesianToJointNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
