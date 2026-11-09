"""Illustrations use camera coordinates and never pretend to measure a pose."""
import math
import unittest
import xml.etree.ElementTree as ET

from openarm_platform.calibration_pose_guide import (
    FIT_POSES, VALIDATION_POSES, guide_step, pose_svg, project)


class PoseGuideTests(unittest.TestCase):
    def test_sequences_roles_and_overflow(self):
        self.assertEqual(len(FIT_POSES),16)
        self.assertEqual(len(VALIDATION_POSES),4)
        self.assertEqual(guide_step('fit',4)[0].yaw,-28)
        self.assertEqual(guide_step('validation',0)[0],VALIDATION_POSES[0])
        self.assertFalse(guide_step('fit',15)[3])
        self.assertTrue(guide_step('fit',16)[3])
        self.assertEqual(guide_step('fit',64)[1:3],(16,16))
        for role,n in [('unknown',0),('fit',-1),('fit',True)]:
            with self.assertRaises(ValueError):guide_step(role,n)

    def test_near_edge_is_larger_in_camera_view_not_a_roll(self):
        def length(pose,a,b):return math.dist(project(pose,*a),project(pose,*b))
        left=((-1.3,-.875),(-1.3,.875));right=((1.3,-.875),(1.3,.875))
        top=((-1.3,-.875),(1.3,-.875));bottom=((-1.3,.875),(1.3,.875))
        self.assertGreater(length(FIT_POSES[4],*left),length(FIT_POSES[4],*right))
        self.assertGreater(length(FIT_POSES[6],*right),length(FIT_POSES[6],*left))
        self.assertGreater(length(FIT_POSES[8],*top),length(FIT_POSES[8],*bottom))
        self.assertGreater(length(FIT_POSES[10],*bottom),length(FIT_POSES[10],*top))
        # No roll: a pure yaw leaves a centre horizontal line horizontal.
        self.assertAlmostEqual(project(FIT_POSES[4],-1,0)[1],project(FIT_POSES[4],1,0)[1])

    def test_svg_is_accessible_self_contained_and_patterns_stay_in_frame(self):
        for pose in (*FIT_POSES,*VALIDATION_POSES):
            svg=pose_svg(pose);root=ET.fromstring(svg)
            self.assertEqual(root.attrib['role'],'img')
            self.assertIn('不是实时检测',root.attrib['aria-label'])
            self.assertNotIn('<script',svg)
            self.assertNotIn('http',svg.split('xmlns="http://www.w3.org/2000/svg"')[1])
            for x,y in [(-1.3,-.875),(1.3,-.875),(1.3,.875),(-1.3,.875)]:
                xx,yy=project(pose,x,y)
                self.assertTrue(8<xx<312,(pose.title,xx))
                self.assertTrue(16<yy<187,(pose.title,yy))
