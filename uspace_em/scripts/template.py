#!/usr/bin/env python

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import *
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from active_perception_controller.srv import ActivePerceptionPlan, ActivePerceptionPlanResponse
import tf
import pdb
import rospy
import roslib
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud, ChannelFloat32
from sklearn.neighbors import NearestNeighbors

import time
import threading
import numpy as np
import scipy as sp
import scipy.ndimage
from active_perception_controller import ap_utility

from StringIO import StringIO
from geometry_msgs.msg._PoseStamped import PoseStamped

class MotionPlanner():
    def __init__(self):        
        self._lock = threading.Lock()
        self._lock.acquire()
        
        self._robot_pose_sub = rospy.Subscriber("amcl_pose",
                                                PoseWithCovarianceStamped,
                                                self.robot_pose_cb,
                                                queue_size=1)
        # TODO: change the particles topic name
        self._target_particles_sub = rospy.Subscriber("person_particle_cloud",
                                                      PointCloud,
                                                      self.target_particle_cloud_cb,
                                                      queue_size=1)

        self._robot_pose = PoseWithCovariance()
        
        self._rrt_pub = rospy.Publisher("rrt",
                                         Path,
                                         queue_size=1,
                                         latch = True)

        self._path_pub = rospy.Publisher("best_path",
                                         Path,
                                         queue_size=1,
                                         latch = True)
        
        self._entropy_pub = rospy.Publisher("entropy_points",
                                         PointCloud,
                                         queue_size=1,
                                         latch = True)

        
        getmap = rospy.ServiceProxy('static_map', GetMap)
        
        srv_available = False
        while not srv_available:
            try:
                rospy.wait_for_service('static_map',2.0)
                srv_available = True
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(e.message)
        
        self._navmap = getmap().map
        width = self._navmap.info.width
        height = self._navmap.info.height
        self._freecells = [i for i in xrange(0,len(self._navmap.data)) 
                           if self._navmap.data[i] == 0]
        
        self._rrt_eta = rospy.get_param("~rrt_eta", 1.0) # Notation from Karaman & Frazolli, 2011
        self._rrt_dist_bias = rospy.get_param("~rrt_total_dist_bias", 0.01)
        self._rrt_near_bias = rospy.get_param("~rrt_nearest_part_bias", 0.1)
        self._rrt_entropy_bias = rospy.get_param("~rrt_entropy_bias", 10)
        robot_radius = rospy.get_param("~robot_radius", 0.5)
        self._robot_radius_px = robot_radius / self._navmap.info.resolution
        sigma_person = rospy.get_param("sigma_person", 0.05)
        self._max_path_size = rospy.get_param("~max_path_size", 10)
        self._max_rrt_iterations = rospy.get_param("~max_rrt_iterations", 200)
        
        #self._planned = False # This is just for testing purposes. Delete me!
        mapdata = np.asarray(self._navmap.data, dtype=np.int8).reshape(height, width)
        logical = np.flipud(mapdata == 0)

        self._distmap = sp.ndimage.distance_transform_edt(logical)

	

        pkgpath = roslib.packages.get_pkg_dir('active_perception_controller')
        self.utility_function = ap_utility.Utility(str(pkgpath) + "/config/sensormodel.png",
                                                   0.050000, sigma_person)
        self.current_weights = []
        self.current_particles = []
        self._plan_srv = rospy.Service('plan', ActivePerceptionPlan, self._plan_srv_cb)
        self._particle_nbrs = NearestNeighbors(n_neighbors=1)
        self._lock.release()
        
    def robot_pose_cb(self, msg):
        self._robot_pose = msg.pose

    def target_particle_cloud_cb(self, msg):
        self._lock.acquire()
        buf = StringIO()
        msg.serialize(buf)
        self.utility_function.setPersonParticles(buf.getvalue())
        channel = [c for c in msg.channels if c.name == 'weights']
        self.current_weights = channel[0].values
        self.current_particles = msg.points
        P = [np.array([0,0])]*len(msg.points)
        for i in xrange(len(msg.points)):
            P[i] = np.array([msg.points[i].x, msg.points[i].y])
        
        self._particle_nbrs.fit(P)
        
        self._lock.release()
        
    def _plan_srv_cb(self, msg):
        path = self.plan()
        res = ActivePerceptionPlanResponse()
        res.path = path
        return res

    def plan(self):
        self._lock.acquire()
        #path = self.rrtstar(self.sample_free_uniform)
        path = self.rrtstar(self.sample_from_particles)
        self._lock.release()
        return path
        
    def rrt(self):
        """
        Basic RRT Algorithm
        """
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])
        V = [probot]
        E = {}
        nbrs = NearestNeighbors(n_neighbors=1)
        nbrs.fit(V)
        t1 = time.time()
        rrt_iter = 0
        while rrt_iter < self._max_rrt_iterations:
            prand = self.sample_free_uniform()
            (dist, idx) = nbrs.kneighbors(prand)
            idx = idx.flatten(1)[0]
            if dist < self._rrt_eta:
                pnew = prand
            else:
                pnew = self.steer(V[idx], prand)
            if self.segment_safe(V[idx],pnew) is True:
                if E.has_key(idx):
                    E[idx].append(len(V))
                else:
                    E[idx] = [len(V)]
                V.append(pnew)
                nbrs.fit(V)
            rrt_iter += 1
        print 'total time: ', time.time()-t1
        self.publish_rrt(V,E) 

    def rrtstar(self, sample_fn):
        """
        RRT* Algorithm
        """
        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        max_range = self.utility_function.getMaximumSensorRange()
        print 'max range is ', max_range
        
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])
        V = [probot]
        E = {}
        parents = {}
        W = [self.current_weights]
        
        w = ap_utility.VectorOfDoubles()
        w_post = ap_utility.VectorOfDoubles()
        w.extend(self.current_weights)
        Ent = [self.utility_function.computeExpEntropy(probot[0], probot[1], 0.0, w, w_post)]
        Dist = [0.0]
        C = [float('Inf')]
        nbrs = NearestNeighbors(n_neighbors=1)
        nbrs.fit(V)
        cmin = 0
        t1 = time.time()
        
        informative_point_found = False
        planning_done = False
        rrt_iter = 0
        
        while not planning_done:
            t2 = time.time()
            """
            Sampling new point
            """
            prand = sample_fn()   
            (dist, idx) = nbrs.kneighbors(prand)
            pnearest_idx = idx.flatten(1)[0]
            pnearest = V[pnearest_idx]

            """
            Turning new point into reachable point
            """
            if dist < self._rrt_eta:
                pnew = prand
            else:
                pnew = self.steer(pnearest, prand)
            """
            Checking if segment is valid and updating graph
            """
            if self.segment_safe(V[pnearest_idx],pnew) is True:
		
                r = np.min([gamma_rrg*sqrt(log(len(V))/float(len(V))),self._rrt_eta])
                Pnear_idx = nbrs.radius_neighbors(pnew, r, return_distance = False)
                Pnear_idx = Pnear_idx[0]
                pmin_idx = pnearest_idx
                w = ap_utility.VectorOfDoubles()
                w_post = ap_utility.VectorOfDoubles()
                w.extend(W[pnearest_idx])

                (dist_nearest_particle, idx) = self._particle_nbrs.kneighbors(pnew)

                if dist_nearest_particle < max_range: #if at least one particle is visible
                    entropy = self.utility_function.computeExpEntropy(pnew[0], pnew[1], 0.0, w, w_post)
                
                if dist_nearest_particle >= max_range or entropy == 0: # utility function failed
                    entropy = Ent[pmin_idx]
                    w_post = w     
                
                dist = np.linalg.norm(pnearest-pnew)
                cmin = (self._rrt_near_bias*dist_nearest_particle +
                        self._rrt_dist_bias * (Dist[pnearest_idx] + dist) +
                        self._rrt_entropy_bias * entropy)

                for p_idx in Pnear_idx:
                    p = V[p_idx]
                    w = ap_utility.VectorOfDoubles()
                    w_near = ap_utility.VectorOfDoubles()
                    w.extend(W[p_idx])
                    
                    if np.abs(Ent[p_idx] - entropy) < 1e-6: # if there is anything to gain in terms of information
                        entropy_near = self.utility_function.computeExpEntropy(pnew[0], pnew[1], 0.0, w, w_near)
                    else:
                        entropy_near = Ent[p_idx]
                    
                    (dist_nearest_particle, idx) = self._particle_nbrs.kneighbors(p)
                    
                    c = (self._rrt_near_bias*dist_nearest_particle + 
                         self._rrt_dist_bias * (Dist[p_idx] + np.linalg.norm(p-pnew)) + 
                         self._rrt_entropy_bias * entropy_near)
                    if (self.segment_safe(p,pnew) is True and 
                        c < cmin):
                        cmin = c
                        pmin_idx = p_idx
                
                if E.has_key(pmin_idx):
                    E[pmin_idx].add(len(V))
                else:
                    E[pmin_idx] = set([len(V)])
                
                pnew_idx = len(V)
                V.append(pnew)
                C.append(cmin)
                W.append(w_post)
                Ent.append(entropy)
                Dist.append(Dist[pmin_idx] + dist)
                parents[pnew_idx] = pmin_idx
                """
                Re-wire the tree
                """
                for p_idx in Pnear_idx:
                    if parents.has_key(p_idx):
                        p = V[p_idx]
                        w = ap_utility.VectorOfDoubles()
                        w_near = ap_utility.VectorOfDoubles()
                        w.extend(W[-1]) # pnew
                        if np.abs(Ent[p_idx] - entropy) > 1e-6: # if there is anything to gain in terms of information
                            entropy_near = self.utility_function.computeExpEntropy(pnew[0], pnew[1], 0.0, w, w_near)
                        else:
                            entropy_near = Ent[p_idx]
                            w_near = w
                        dist = np.linalg.norm(p-pnew)
                        (dist_nearest_particle, idx) = self._particle_nbrs.kneighbors(p)
                         
                        c = (self._rrt_near_bias*dist_nearest_particle + 
                             self._rrt_dist_bias * (Dist[-1] + dist) + 
                             self._rrt_entropy_bias * entropy_near)
                        if (self.segment_safe(p,pnew) is True and 
                            c < C[p_idx]):
                            E[parents[p_idx]].remove(p_idx)
                            parents[p_idx] = pnew_idx
                            print 'rewired ',p_idx,'to',pnew_idx
                            if E.has_key(pnew_idx):
                                E[pnew_idx].add(p_idx)
                            else:
                                E[pnew_idx] = set([p_idx])
                            C[p_idx] = c
                            W[p_idx] = w_near
                            Ent[p_idx] = entropy_near
                            Dist[p_idx] = Dist[-1] + dist
                nbrs.fit(V)
            # print 'iteration done. time: ', time.time()-t2
            # print 'min entropy:', np.min(I)
            if np.max(Ent) - np.min(Ent) > 1e-6: # just to compensate arithmetic noise
                informative_point_found = True

            """
            Find best path:
            """
            
            path = self.get_best_path(parents, V, C)

            if informative_point_found and len(path.poses) > self._max_path_size:
                planning_done = True

            rrt_iter += 1
            
            if rrt_iter > self._max_rrt_iterations:
                planning_done = True
                if not informative_point_found:
                    rospy.logwarn("Could not find an informative goal point in %d iterations! Aborting.", self._max_rrt_iterations)
                    
        print 'total time: ', time.time()-t1
        
        self.publish_rrt(V,E)
        
        self._path_pub.publish(path)
        
        self.publish_entropy_info(V, Ent)
        return path

    def publish_rrt(self, V,E):
        pt = Path()
        pt.header.frame_id = '/map'
        path = []
        vis = set()
        self.gen_path(0, 0, V, E, path, vis)
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = '/map'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pt.poses.append(pose)
            last_pose = pose
        self._rrt_pub.publish(pt)
    
    def get_best_path(self, parents, V, C):
        pt = Path()
        pt.header.frame_id = '/map'
        m = np.argmin(C)
        at_root = False
        c = 0
        
        last_pose = PoseStamped()
        
        while not at_root and c < len(C):
            pose = PoseStamped()
            pose.header.frame_id = '/map'
            # pose.header.seq = c
            pose.header.stamp = rospy.Time.now()
            p = V[m]
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            th = atan2(pose.pose.position.y - last_pose.pose.position.y,
                       pose.pose.position.x - last_pose.pose.position.x)
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0 #sin(th/2.0) #0
            pose.pose.orientation.w = 1 #cos(th/2.0) #1
            pt.poses.append(pose)
            
            if m == 0:
                at_root = True
            else:
                m = parents[m]
            c += 1
        if not at_root:
            rospy.logerr("Could not find RRT root! Exiting at node %d",m)
            #pdb.set_trace()
            
        pt.poses.reverse() # fundamental, since poses were added from the end to the beginning
        return pt

    def publish_entropy_info(self, V, Ent):
        pc = PointCloud()
        ch = ChannelFloat32()
        ch.name = 'weights'
        ch.values = Ent
        for v in V:
            g = Point32()
            g.x = v[0]
            g.y = v[1]
            pc.points.append(g)
        pc.channels.append(ch)
        pc.header.frame_id = "/map"
        self._entropy_pub.publish(pc)
        
    def gen_path(self, ix, p_ix, V, E, path, vis ):
        path.append(V[ix])
        vis.add(ix)
        if E.has_key(ix):
            for c in E[ix]:
                if c not in vis:
                    self.gen_path(c, ix, V, E, path, vis )
        path.append(V[p_ix])
   
    def steer(self, org, dst):
        alpha = atan2(dst[1]-org[1],
                      dst[0]-org[0])
        new = org + self._rrt_eta*np.array([cos(alpha),sin(alpha)])
        return new
                
    def segment_safe(self, org, dst):
        xo = self._navmap.info.origin.position.x
        yo = self._navmap.info.origin.position.y
        res = self._navmap.info.resolution
	height=self._navmap.info.height*res

       # org_idx = np.array([int((org[0] - xo)/res),int(-(org[1] + yo)/res)])
       # dst_idx = np.array([int((dst[0] - xo)/res),int(-(dst[1] + yo)/res)])
        org_idx = np.array([int((org[0] - xo)/res),int((height+yo-org[1])/res)])
        dst_idx = np.array([int((dst[0] - xo)/res),int((height+yo-dst[1])/res)])

        # t1 = time.time()
        v1 = self.distance_transform_search(org_idx, dst_idx)
        # t2 = time.time()
        # v2 = self.greedy_cardinal_search(org_idx, dst_idx)
        # t3 = time.time()
        # v3 = self.directed_search(org_idx, dst_idx)
        # t4 = time.time()
        # print '----------------------------------------'
        # print 'dist transform: ',v1,' t: ',t2-t1
        # print 'greedy search: ',v2,' t: ',t3-t2
        # print 'directed search: ',v3,' t: ',t4-t3
        return v1
    
    def distance_transform_search(self, org_idx, dst_idx):
	#print self._distmap[dst_idx[1],dst_idx[0]]
	#print self._robot_radius_px
        if self._distmap[dst_idx[1],dst_idx[0]] < self._robot_radius_px:
            return False
        
        alpha = atan2(dst_idx[1]-org_idx[1],
                      dst_idx[0]-org_idx[0])
        ca = cos(alpha)
        sa = sin(alpha)
        ridx = org_idx
        while not np.all(ridx == dst_idx):
            dist = self._distmap[int(ridx[1]),
                                 int(ridx[0])]
            if dist < self._robot_radius_px:
                return False
            elif np.linalg.norm(ridx - dst_idx) < dist:
                return True
            else:
                ridx = ridx + np.array([ca, sa])*dist
        return True
    
    def directed_search(self, org_idx, dst_idx):
        alpha = atan2(dst_idx[1]-org_idx[1],
                      dst_idx[0]-org_idx[0])
        ca = cos(alpha)
        sa = sin(alpha)   
        idx = org_idx
        ridx = idx
        while not np.all(ridx == dst_idx):
            idx = idx + np.array([ca, sa])
            ridx = np.floor(idx)
            linear_idx = int(((self._navmap.info.height-ridx[1]-1)*self._navmap.info.width
                              + ridx[0]))
            if self._navmap.data[linear_idx] != 0:
                return False
        return True
    
    def greedy_cardinal_search(self, org_idx, dst_idx, prev_idx = None): 
        if np.all(org_idx == dst_idx):
            return True
      
        dir = np.mat('1 0; -1 0; 0 1; 0 -1; 1 1; -1 -1; 1 -1; -1 1')
         
        next = org_idx + dir
        norms = np.linalg.norm(next - dst_idx, 2, 1)
        best_idx = np.asarray(next[np.argmin(norms),:]).flatten(1)
        best_linear_idx = ((self._navmap.info.height-best_idx[1]-1)*self._navmap.info.width
                           + best_idx[0])
        if self._navmap.data[best_linear_idx] == 0:
            return self.greedy_cardinal_search(best_idx, dst_idx, org_idx)
        else:
            return False
        
    def sample_free_uniform(self):
        r = np.random.uniform(0,len(self._freecells),1)
        idx = self._freecells[int(r)]
        xo = self._navmap.info.origin.position.x
        yo = self._navmap.info.origin.position.y
        res = self._navmap.info.resolution
        w = self._navmap.info.width
        h = self._navmap.info.height
        x = (idx%w)*res+xo
        y = ((idx)/w)*res+yo
        return np.array([x,y])
        
    def sample_from_particles(self):
        r = np.random.uniform(0,len(self.current_particles),1)
        p = self.current_particles[int(r)]
        return np.array([p.x, p.y])
        
if __name__=='__main__':
    rospy.init_node('entropy_motion_planner')
    m = MotionPlanner()
    
    rospy.spin()