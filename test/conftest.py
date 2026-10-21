"""Keep test discovery separate from the operational robot graph."""
import os

os.environ['ROS_DOMAIN_ID'] = '217'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
