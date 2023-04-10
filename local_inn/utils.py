import json
import numpy as np
import torch
import tf_transformations
import yaml

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped


class ConfigJSON():
    def __init__(self) -> None:
        self.d = {}
    
    def load_file(self, filename):
        with open(filename, 'r') as f:
            self.d = json.load(f)
    
    def save_file(self, filename):
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.d, f, ensure_ascii=False, indent=4)
            
            
class DataProcessor():
    def __init__(self) -> None:
        pass
    
    def two_pi_warp(self, angles):
        twp_pi = 2 * np.pi
        return (angles + twp_pi) % (twp_pi)
        # twp_pi = 2 * np.pi
        # if angle > twp_pi:
        #     return angle - twp_pi
        # elif angle < 0:
        #     return angle + twp_pi
        # else:
        #     return angle
    
    def data_normalize(self, data):
        data_min = np.min(data)
        data = data - data_min
        data_max = np.max(data)
        data = data / data_max
        return data, [data_max, data_min]
    
    def runtime_normalize(self, data, params):
        return (data - params[1]) / params[0]
    
    def de_normalize(self, data, params):
        return data * params[0] + params[1]
    
def mmd_multiscale(x, y):
    xx, yy, zz = torch.mm(x, x.t()), torch.mm(y, y.t()), torch.mm(x, y.t())

    rx = (xx.diag().unsqueeze(0).expand_as(xx))
    ry = (yy.diag().unsqueeze(0).expand_as(yy))

    dxx = rx.t() + rx - 2.*xx
    dyy = ry.t() + ry - 2.*yy
    dxy = rx.t() + ry - 2.*zz

    XX, YY, XY = (torch.zeros(xx.shape).to('cuda'),
                  torch.zeros(xx.shape).to('cuda'),
                  torch.zeros(xx.shape).to('cuda'))

    # kernel computation
    for a in [0.05, 0.2, 0.9]:
        XX += a**2 * (a**2 + dxx)**-1
        YY += a**2 * (a**2 + dyy)**-1
        XY += a**2 * (a**2 + dxy)**-1

    return torch.mean(XX + YY - 2.*XY)

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    q = tf_transformations.quaternion_from_euler(0, 0, angle)
    q_out = Quaternion()
    q_out.x = q[0]
    q_out.y = q[1]
    q_out.z = q[2]
    q_out.w = q[3]
    return q_out

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))
    return yaw

def rotation_matrix(theta):
    ''' Creates a rotation matrix for the given angle in radians '''
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])

def particle_to_pose(particle):
    ''' Converts a particle in the form [x, y, theta] into a Pose object '''
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(particle[2])
    return pose

def particles_to_poses(particles):
    ''' Converts a two dimensional array of particles into an array of Poses. 
        Particles can be a array like [[x0, y0, theta0], [x1, y1, theta1]...]
    '''
    return list(map(particle_to_pose, particles))

class DrivableCritic():
    def __init__(self, yaml_filename):
        with open(yaml_filename) as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)
        self.img = np.load(yaml_filename.split('/')[0] + '/' + map_yaml['image'] + '.npy')
        # self.img = cv2.imread(map_yaml['image'], cv2.IMREAD_GRAYSCALE)
        self.img_ori = np.array(map_yaml['origin'])
        self.img_res = np.array(map_yaml['resolution'])
        self.x_in_m = self.img.shape[1] * self.img_res
        self.y_in_m = self.img.shape[0] * self.img_res
        
    def get_normalize_params(self):
        params = np.zeros((4, 2))
        params[0, 1] = self.img_ori[0]
        params[0, 0] = self.x_in_m
        params[1, 1] = self.img_ori[1]
        params[1, 0] = self.y_in_m
        params[2, 0] = np.pi * 2
        params[3, 0] = 30
        return params
        
    def pose_2_colrow(self, pose):
        colrow = (pose[:, :2] - self.img_ori[:2]) / self.img_res
        return np.int32(colrow)

    def normalized_pose_2_rowcol(self, normalized_pose):
        return np.int32([(1 - normalized_pose[:, 1]) * self.img.shape[0],
                  normalized_pose[:, 0] * self.img.shape[1]]).transpose(1, 0)

    def normalized_pose_find_drivable(self, normalized_pose):
        rowcol = self.normalized_pose_2_rowcol(normalized_pose)
        return self.img[rowcol[:, 0], rowcol[:, 1]]
    
    def normalized_pose_2_xy(self, normalized_pose):
        img_size_xy = np.array([self.img.shape[1], self.img.shape[0]])
        return (normalized_pose * img_size_xy) * self.img_res + self.img_ori[:2]