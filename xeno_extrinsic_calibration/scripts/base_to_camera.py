#!/usr/bin/env python


'''
@author Jose Alvarez-Ruiz
@date   June 2016
'''

import rospy
import tf
import tf.transformations as tlib
import yaml
import time
import datetime

tf_listener= None


def shutdown_hook(trans):    
    while True:
        print 'T= (%f, %f, %f) E= (%f, %f, %f)' % \
        (trans['x'], trans['y'], trans['z'], trans['yaw'], trans['pitch'], trans['roll'])
        ans= raw_input( 'Save transformation from  base_link to stereo1_left? [y/n] ')
        print 'Answer', ans
        if ans not in ('y', 'n'):
            continue
        if ans == 'y':
            with open('/home/rosstuhl/extrinsics1.yaml', 'w') as f:
                yaml.dump(trans, f, default_flow_style= False)

            with open('/home/rosstuhl/base_link_to_stereo1_left.xml', 'w' ) as f:
                print >> f, '''
<!-- This launch file was generated automatically, and might be modified likewise. -->
<launch>
   <node pkg="tf" type="static_transform_publisher" name="%s_to_%s_transform_publisher"
     args="%f %f %f %f %f %f %s %s 100"
    />
</launch>
''' % ('base_link', 'stereo1_left',
        trans['x'], trans['y'], trans['z'],
        trans['yaw'], trans['pitch'], trans['roll'],
        'base_link',
        'stereo1_left')
        return

def main():
    br = tf.TransformBroadcaster()
    rate= rospy.Rate(5)
    pbm= None # Position of dummy marker w.r.t. base_link
    qbm= None # Orientation of dummy marker w.r.t. base_link
    pms= None # Position of marker w.r.t. camera
    qms= None # Orientation of dummy marker w.r.t. camera
    found= False
    while not rospy.is_shutdown():
        #send_identity()
        try:
            found= False
            delay= rospy.Time.now() - tf_listener.getLatestCommonTime('ar_marker_0', 'stereo1_left')
            if delay.to_sec() > 1:
                rospy.loginfo("Marker not found")
                continue
            found= True
            tf_listener.waitForTransform("ar_marker_0", "stereo1_left", rospy.Time(),
                                         rospy.Duration(4.0))
            tf_listener.waitForTransform("base_link", "ar_marker_0_dummy", rospy.Time(),
                                         rospy.Duration(4.0))
            pbm, qbm= tf_listener.lookupTransform("base_link", "ar_marker_0_dummy",
                                                  rospy.Time())
            pms, qms= tf_listener.lookupTransform("ar_marker_0", "stereo1_left",
                                                  rospy.Time())
            base_to_dummy_transform=tlib.compose_matrix(angles= tlib.euler_from_quaternion(qbm),
                                             translate= pbm)
            marker_to_stereo_transform=tlib.compose_matrix(angles= tlib.euler_from_quaternion(qms),
                                                 translate= pms)
    
            joint_transform_matrix= tlib.concatenate_matrices(base_to_dummy_transform, marker_to_stereo_transform)    
            joint_translation= tlib.translation_from_matrix(joint_transform_matrix)
            joint_euler= tlib.euler_from_matrix(joint_transform_matrix)          
            br.sendTransform(joint_translation,
                      tlib.quaternion_from_euler(*joint_euler),
                      rospy.Time.now(),
                      "stereo1_left",
                      "base_link")
            print 'Marker found with transform', pbm, qbm
            rate.sleep()
        except tf.Exception, e:
            print 'Marker not found'
    trans= {
        'frame_id' : 'base_link',
        'child_id' : 'stereo1_left',
        'x' : float(joint_translation[0]),
        'y' : float(joint_translation[1]),
        'z' : float(joint_translation[2]),
        'roll' : float(joint_euler[0]),
        'pitch' : float(joint_euler[1]),
        'yaw' : float(joint_euler[2]),
        
    }
    rospy.on_shutdown(lambda: shutdown_hook(trans))

def validate():
    frames= ("base_link", "ar_marker_0_dummy")    
    for f in frames:
        rospy.loginfo("Veryfing that frame %s exists!" % f)
        if not tf_listener.frameExists(f):
            rospy.logerr("frame %s does not exist" % f)
            return False
    try:
        rospy.loginfo("Veryfing that transform does NOT exists!")
        t = tf_listener.getLatestCommonTime("base_link", "stereo1_left")
        return False
    except tf.Exception, e:
        pass

    rospy.loginfo("Veryfing that marker is connected to base_link!")
    tf_listener.waitForTransform("ar_marker_0_dummy", "base_link", rospy.Time(), rospy.Duration(4.0))
    return True


if __name__ == '__main__':
    rospy.init_node('calibration_base_to_stereo')
    tf_listener= tf.TransformListener()
    rospy.loginfo("Waiting for transform info...")
    rospy.Rate(5).sleep()
    rate = rospy.Rate(1.0)
    if not validate():
        rospy.signal_shutdown('Missing frames')
    main()
