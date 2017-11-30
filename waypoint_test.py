#!/usr/bin/env python

import argparse
import sys
import rospy
import time

import baxter_interface

from baxter_interface import CHECK_VERSION    

from geometry_msgs.msg import ( #necessary to build the request message for the IK service
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#valid=True;
pointlist=[];

def get_joint_angles(limb,point):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"   #set service name ns
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)   #Initilaze Object by passing the service name ns and the Request-Response message type, 
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
    'left': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(x = point[0],y = point[1],z = point[2],),
            orientation=Quaternion(x = point[3], y = point[4],z = point[5],w = point[6], 
           
            ),
        ),
    ),
    'right': PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(x = point[0],y = point[1],z = point[2],),
            orientation=Quaternion(x = point[3], y = point[4],z = point[5],w = point[6],
            ),
        ),
    ),
    } 
   
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return (limb_joints)
    else:
        #print("INVALID POSE - No Valid Joint Solution Found.")
        valid=False;
    return 0
	

def clean_shutdown(self):
        print("\nExiting program...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True    

def callback(msg):
	global pointlist;
	final_point=msg
	pointlist=([final_point.pose.position.x, final_point.pose.position.y, final_point.pose.position.z+0.075, 0.982093 , -0.174111 , 0.052059 , -0.049695]);
	#pointlist=[final_point.point.x, final_point.point.y, final_point.point.z+0.100, 0.982093 , -0.174111 , 0.052059 , -0.049695];
	#pointlist=[0.754, 0.152, -0.1046, 0.982093 , -0.174111 , 0.052059 , -0.049695]
	#pointlist=[0.794, 0.172, -0.1320, 0.982093 , -0.174111 , 0.052059 , -0.049695]

def main():
    

    global pointlist;
    rospy.init_node('waypoint_test', anonymous=True)  #initialize node..
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)

    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )

    parser.add_argument(
        '-s', '--speed', default=0.3, type=float,
        help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
    )
    parser.add_argument(
        '-a', '--accuracy',
        default=baxter_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
        help='joint position accuracy (rad) at which waypoints must achieve'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    #return ik_test(args.limb)
    

    #limb_joints = list();
    #if(args.limb=="left"):
    	#pointlist.append([0.657579481614,0.851981417433,0.0388352386502,-0.367048116303, 0.885911751787, 0.108908281936, 0.261868353356]);
    #else:
	
	#off = 0.075;
	


        #pointlist.append([0.754, 0.152, -0.1046, 0.982093 , -0.174111 , 0.052059 , -0.049695]);
	#pointlist.append([0.840275 , -0.124160 , 0.091155,0.982093 , -0.174111 , 0.052059 , -0.049695]);

    
  

   
	
    
    
	
    print("Getting robot state... ")
    rs=baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    arm = baxter_interface.Limb(args.limb)
   	
    arm.set_joint_position_speed(args.speed)
   
    rospy.Subscriber("/finalpts", PoseStamped, callback)

   
    _limb = baxter_interface.Limb(args.limb)
    while not rospy.is_shutdown():
	 
	 """for point in pointlist:
        	joint_angle= get_joint_angles(args.limb,point)
	 	if(joint_angle):
	    	      limb_joints.append(joint_angle)"""

	 #print(pointlist);
	 #print(len(pointlist));
	 if(len(pointlist)==7):
		 joint_angle= get_joint_angles(args.limb,pointlist)
	 	 #for waypoint in limb_joints:
		 if rospy.is_shutdown():
		          break
		 #print("Point:  %d" %limb_joints.index(waypoint));
		 #print("Point:  %d" %joint_angle)
		 #arm.move_to_joint_positions(waypoint, timeout=20.0, threshold=args.accuracy)
		 if(joint_angle):
		 	  arm.move_to_joint_positions(joint_angle, timeout=20.0, threshold=args.accuracy)
		 baxter_pose=_limb.endpoint_pose()["position"]
            	 orient=_limb.endpoint_pose()["orientation"]
	   	 print "Baxter end_point:  %f , %f , %f"%(baxter_pose.x,baxter_pose.y,baxter_pose.z)
            	 print "orientation:  %f , %f , %f , %f"%(orient.x,orient.y,orient.z, orient.w)   	
		 """baxter_pose=arm.endpoint_pose()["position"]
		 orient=arm.endpoint_pose()["orientation"]
		 print "Baxter end_point:  %f , %f , %f"%(baxter_pose.x,baxter_pose.y,baxter_pose.z)
		 print "\n"
		 print "orientation:  %f , %f , %f , %f"%(orient.x,orient.y,orient.z, orient.w)
		 #time.sleep(1);"""
		 rospy.sleep(0.01)
		# Set joint position speed back to default
    arm.set_joint_position_speed(0.3)
	
    
        	


if __name__ == '__main__':
    sys.exit(main())


