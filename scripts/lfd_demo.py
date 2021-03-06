#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import matplotlib.pyplot as plt

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 2-D trajectory
    dims = 2                
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 100          
    traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.0,0.0]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0]   
    t_0 = 0                
    goal = [8.0,7.0]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    
    num_pts  = len(plan.plan.points)+1
    positions= [ [ 0 for i in range(num_pts) ] for j in range(dims) ]
    positions[0][0] = x_0[0]
    positions[1][0] = x_0[1]
    #print plan
    for i in xrange(1,num_pts):
        positions[0][i] = plan.plan.points[i-1].positions[0]
        positions[1][i] = plan.plan.points[i-1].positions[1]

    times = plan.plan.times
    times = (0,)+times

    num_dem_pts = len(traj)
    traj_arr = [ [ 0 for i in range(num_dem_pts) ] for j in range(dims) ]
    for i in xrange(0,num_dem_pts):
        traj_arr[0][i] = traj[i][0]
        traj_arr[1][i] = traj[i][1]

    print positions
    print plan

    plt.plot(times,positions[0],'r')
    #plt.plot(times,positions[1])
    plt.plot(times[0:num_dem_pts],traj_arr[0],'g')
    plt.ylabel('new_plan')
    plt.show()
