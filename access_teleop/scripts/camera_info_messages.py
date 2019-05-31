from sensor_msgs.msg import CameraInfo
import rospy

camera1 = CameraInfo()
camera2 = CameraInfo()
camera3 = CameraInfo()
"""
camera1 = {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'camera1'},
      height: 342, width: 342, distortion_model: 'plumb_bob',
      D: [0],
      K: [500.0, 0.0, 320, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
      R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
      P: [500.0, 0.0, 320, 0.0, 0.0, 500, 240, 0.0, 0.0, 0.0, 1.0, 0.0],
      binning_x: 0, binning_y: 0,
      roi: {x_offset: 0, y_offset: 0, height: 480, width: 640, do_rectify: false}}
"""
camera1.header.seq = 0
camera1.header.stamp.secs = 0
camera1.header.stamp.nsecs = 0
camera1.header.frame_id = 'camera1'
camera1.height = 480
camera1.width = 640
camera1.distortion_model = 'plumb_bob'
camera1.D = [0]
camera1.K = [500.0, 0.0, 320, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
camera1.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera1.P = [500.0, 0.0, 320, 0.0, 0.0, 500, 240, 0.0, 0.0, 0.0, 1.0, 0.0]
camera1.binning_x = 0
camera1.binning_y = 0
camera1.roi.x_offset = 0
camera1.roi.y_offset = 0
camera1.roi.height = 480
camera1.roi.width = 640
camera1.roi.do_rectify = False

"""
camera2 = {header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'camera2'},

      height: 480, width: 640, distortion_model: 'plumb_bob',

      D: [0],
      K: [300.0, 0.0, 640, 0.0, 300.0, 360.0, 0.0, 0.0, 1.0],
      R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
      P: [300.0, 0.0, 640, 0.0, 0.0, 300, 360, 0.0, 0.0, 0.0, 1.0, 0.0],
      binning_x: 0, binning_y: 0,
      roi: {x_offset: 0, y_offset: 0, height: 720, width: 1280, do_rectify: false}}
"""
camera2.header.seq = 0
camera2.header.stamp.secs = 0
camera2.header.stamp.nsecs = 0
camera2.header.frame_id = 'camera2'
camera2.height = 480
camera2.width = 640
camera2.distortion_model = 'plumb_bob'
camera2.D = [0]
camera2.K = [500.0, 0.0, 320, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
camera2.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera2.P = [500.0, 0.0, 320, 0.0, 0.0, 500, 240, 0.0, 0.0, 0.0, 1.0, 0.0]
camera2.binning_x = 0
camera2.binning_y = 0
camera2.roi.x_offset = 0
camera2.roi.y_offset = 0
camera2.roi.height = 480
camera2.roi.width = 640
camera2.roi.do_rectify = False


camera3.header.seq = 0
camera3.header.stamp.secs = 0
camera3.header.stamp.nsecs = 0
camera3.header.frame_id = 'camera3'
camera3.height = 480
camera3.width = 640
camera3.distortion_model = 'plumb_bob'
camera3.D = [0]
camera3.K = [500.0, 0.0, 320, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
camera3.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
camera3.P = [500.0, 0.0, 320, 0.0, 0.0, 500, 240, 0.0, 0.0, 0.0, 1.0, 0.0]
camera3.binning_x = 0
camera3.binning_y = 0
camera3.roi.x_offset = 0
camera3.roi.y_offset = 0
camera3.roi.height = 480
camera3.roi.width = 640
camera3.roi.do_rectify = False