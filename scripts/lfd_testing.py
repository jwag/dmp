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

    #Create a DMP from a 1-D trajectory
    dims = 1                
    dt = 0.1                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 100          
    w = np.pi/2;
    num_dem_pts = 100;
    traj = [[ np.cos(w*dt*j) for i in range(dims) ]for j in range(num_dem_pts) ]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    #x_0 = [1.0]          #Plan starting at a different point than demo 
    #x_dot_0 = [0.0]   
    x_0 = [traj[0][0]] 
    x_dot_0 = [0.0]
    t_0 = 0                
    #goal = [0.0]         #Plan to a different goal than demo
    #goal = [0.95105];
    #goal = [-2.0]
    goal = [traj[num_dem_pts-1][0]]
    #print traj[num_dem_pts-1][0]
    goal_thresh = [0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 1 * resp.tau       #Desired plan should take twice as long as demo
    dt = 0.1
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    
    num_pts  = len(plan.plan.points)+1
    positions= [ [ 0 for i in range(num_pts) ] for j in range(dims) ]
    positions[0][0] = x_0[0]
    #print plan
    for i in xrange(1,num_pts):
        positions[0][i] = plan.plan.points[i-1].positions[0]

    times = plan.plan.times
    times = (0,)+times

    traj_arr = [[ 0 for i in range(num_dem_pts) ]for j in range(dims) ]
    for i in xrange(0,num_dem_pts):
        traj_arr[0][i] = traj[i][0]

    #print positions
    #print plan

    # plt.plot(times[0:num_dem_pts-90],positions[0][0:num_dem_pts-90],'r')
    plt.plot(times,positions[0],'r')
    #plt.plot(times,positions[1])
    demo_time = [ i*dt for i in range(num_dem_pts) ]
    plt.plot(demo_time,traj_arr[0],'g')
    #plt.plot(demo_time,resp.dmp_list[0].f_targets,'b')
    plt.ylabel('new_plan')
    plt.show()
