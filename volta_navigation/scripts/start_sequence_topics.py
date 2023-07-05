#!/usr/bin/python
import rospy
import time
import rospkg
import tf
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose,Twist, Vector3
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from volta_navigation.srv import Goal, GoalResponse
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import yaml

class Sequence:

    def __init__(self):
        # Init the sequence node 
        rospy.init_node("sequence")

        # Start Service
        # rospy.Service('goal_server', Goal, self.handle_goal)

        # Load Params for Navigation
        rospack = rospkg.RosPack()
        path = rospack.get_path('volta_navigation')
        path += '/scripts/param.yaml'
        params = yaml.safe_load(open(path))

        # Initialize variables 
        self.goal = []
        self.yaw = 0
        self.scan_data = []
        self.lidar_safe = True
        self.lidar_safe_yaw = True

        # MoveBase ActionClient 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Initialize Publishers
        self.marker_pub = rospy.Publisher('goal_server_goal', Marker , queue_size=1, latch=True)
        self.marker_pub2 = rospy.Publisher('recieved_goal', Marker , queue_size=1, latch=True)
        self.vel_pub = rospy.Publisher('/goal_server_vel/cmd_vel', Twist, queue_size=1, latch=True)

        self.params = params

        self.shift_goal = params['shift_goal']

        self.lidar_safe_dist = params['lidar_safe_dist']

        self.lidar_safe_dist_rotate = params['lidar_safe_dist_rotate']

        self.clearance_yaw = params['clearance_yaw']
        
        x_max =  params['x_max']
        x_min =  params['x_min']
        
        y_max =  params['y_max']
        y_min =  params['y_min']

        self.center_arena = (x_max + x_min)/2 , (y_max + y_min)/2


        x_max_offset =  params['x_max_offset']
        x_min_offset =  params['x_min_offset']

        y_max_offset =  params['y_max_offset']
        y_min_offset =  params['y_min_offset']

        self.front_angle = params['front_angle']

        self.clearance = params['clearance']

        factor = params['corner_factor']

        self.soft_xmax = x_max - x_max_offset
        self.soft_xmin = x_min + x_min_offset

        self.soft_ymax = y_max - y_max_offset
        self.soft_ymin = y_min + y_min_offset

        self.corner_xmax = x_max - x_max_offset * factor
        self.corner_xmin = x_min + x_min_offset * factor

        self.corner_ymax = y_max - y_max_offset * factor
        self.corner_ymin = y_min + y_min_offset * factor

        
        
        self.response_pub = rospy.Publisher('/response', Bool, queue_size=1, latch=True)

        resp = Bool()
        resp.data = False
        self.response_pub.publish(resp)

        # Scan Subscriber
        rospy.Subscriber('/scan_filtered', LaserScan, self.laser_callback)
        rospy.Subscriber('/goals', Pose, self.handle_goal)
        rospy.spin()        

    def marker_publish(self,pub):
        m = Marker()
        m.action = Marker.ADD
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time(0)
        m.ns = 'marker_test_%d' % Marker.ARROW
        m.id = 0
        m.type = Marker.ARROW
        m.pose.position.x = self.goal[0]
        m.pose.position.y = self.goal[1]
        m.pose.orientation.z = math.sin(self.yaw/2)
        m.pose.orientation.w = math.cos(self.yaw/2)
        m.scale = Vector3(0.3,0.06,0.06)
        m.color.r = 0
        m.color.g = 0
        m.color.b = 0
        m.color.a = 1.0

        if pub==1:
            m.color.b = 1
            self.marker_pub.publish(m)
        if pub==2:
            m.color.r = 1
            self.marker_pub2.publish(m)

    def laser_callback(self, msg):

        factor = int(len(msg.ranges)/360)

        middle = self.front_angle
        clear = self.clearance

        min_dist = min(msg.ranges[ (middle-clear) * factor: (middle+clear) * factor])
           
        self.lidar_safe = True if min_dist > self.lidar_safe_dist else False

        if middle == 0:
            min_dist = min( min(msg.ranges[ -clear * factor: -1]), min(msg.ranges[ 0 : clear * factor ]) )
        else:
            min_dist = min(msg.ranges[ (middle-clear) * factor: (middle+clear) * factor])
           
        self.lidar_safe = True if min_dist > self.lidar_safe_dist else False

        clear = self.clearance_yaw # 20

        lidar_min_dist = min( min(msg.ranges[ (90 - clear) * factor: (90 + clear) * factor]) , min(msg.ranges[ (- 90 - clear) * factor: ( - 90 + clear) * factor]) )

        self.lidar_safe_yaw = True if lidar_min_dist > self.lidar_safe_dist_rotate else False
        

    def get_velocity(self,distance):

        return 1/(1 + math.exp(-10*distance))

    def get_orientation(self):

        x, y = self.goal

        if x > self.corner_xmax :

            self.yaw = 0

            if y > self.corner_ymax :

                self.yaw = math.pi / 4

            elif y < self.corner_ymin :

                self.yaw = - math.pi / 4

        elif x < self.corner_xmin :

            self.yaw = math.pi

            if y > self.corner_ymax :

                self.yaw =  3 * math.pi / 4

            elif y < self.corner_ymin :

                self.yaw = - 3 *  math.pi / 4

        else:

            if y > self.corner_ymax :

                self.yaw = math.pi / 2

            elif y < self.corner_ymin :

                self.yaw = - math.pi / 2


    def plan_to_goal(self):

        new_goal_x , new_goal_y = self.goal[0] - self.shift_goal * math.cos(self.yaw) , self.goal[1] - self.shift_goal * math.sin(self.yaw)
        
        return new_goal_x , new_goal_y

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        # return roll_x, pitch_y, yaw_z # in radians
        return yaw_z

    def approach_yaw(self):

        listener = tf.TransformListener()

        threshold = 0.08
        diff = threshold
        
        rate = rospy.Rate(10.0)

        while  not rospy.is_shutdown() and  diff >= threshold and self.lidar_safe_yaw :
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            cur_yaw = self.euler_from_quaternion(rot[0],rot[1],rot[2],rot[3])

            # angular =  (int((self.yaw - cur_yaw)*180/math.pi)%360)*(math.pi/180) 

            angular = (self.yaw - cur_yaw) if (self.yaw - cur_yaw) < (2 * math.pi - (self.yaw - cur_yaw)) else  -(2 * math.pi - (self.yaw - cur_yaw))
        
            # angular =  min( (self.yaw - cur_yaw) , (2 * math.pi - (self.yaw - cur_yaw)))

            print(self.yaw)
            print( "First: ", (self.yaw - cur_yaw) ," Second:" ,(2 * math.pi - (self.yaw - cur_yaw)) )
       
            print("Angular",angular)
            cmd = Twist()
            cmd.angular.z = angular
            self.vel_pub.publish(cmd)

            diff = min( abs((self.yaw - cur_yaw)) , abs((2 * math.pi - (self.yaw - cur_yaw))))

            rate.sleep()

        cmd = Twist()
        cmd.angular.z = 0

        self.vel_pub.publish(cmd)

    
    def approach_goal(self):

        listener = tf.TransformListener()
        while True:
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        init_x, init_y = trans[0], trans[1]

        threshold = math.sqrt( (init_x - self.goal[0] )**2 + (init_y - self.goal[1] )**2  )
        
        distance_moved = 0

        rate = rospy.Rate(10.0)

        while  not rospy.is_shutdown() and self.lidar_safe:
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Can't find transform"
                continue

            cur_x, cur_y = trans[0], trans[1]
        
            linear = 0.08 * (threshold - distance_moved)/threshold
            # linear = 0.05 * self.get_velocity(distance_moved)
            # linear = 0.1
            # print("linea",linear)
            cmd = Twist()
            cmd.linear.x = linear
            self.vel_pub.publish(cmd)

            distance_moved= math.sqrt( (init_x-cur_x)**2 + (init_y-cur_y)**2 )

            rate.sleep()

        cmd = Twist()
        cmd.linear.x = 0

        self.vel_pub.publish(cmd)

    def retreat(self, threshold = 0.6):

        listener = tf.TransformListener()
        while True:
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        init_x, init_y = trans[0], trans[1]

        # threshold = math.sqrt( (init_x - self.goal[0] )**2 + (init_y - self.goal[1] )**2  )
        threshold = 0.3
        
        distance_moved = 0

        rate = rospy.Rate(10.0)

        while  not rospy.is_shutdown() and  distance_moved <= threshold :
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            cur_x, cur_y = trans[0], trans[1]
        
            linear = -0.15 * self.get_velocity(threshold - distance_moved)
            cmd = Twist()
            cmd.linear.x = linear
            self.vel_pub.publish(cmd)

            distance_moved= math.sqrt( (init_x-cur_x)**2 + (init_y-cur_y)**2 )
            # print("Curr:",[cur_x,cur_y])
            # print(distance_moved)

            rate.sleep()

        cmd = Twist()
        cmd.linear.x = 0

        self.vel_pub.publish(cmd)

        # print("Final",cur_x,cur_y)
    
    def movebase_client(self,x,y):

        
        print "\n[move_base] Waiting for MoveBase server \n"
        self.client.wait_for_server()
        print "\n[move_base] MoveBase Server Up!\n"
        print "\n[move_base] Sending MoveBase Goal :[ %s , %s , %s] \n"%(x, y, self.yaw)

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = x
        move_base_goal.target_pose.pose.position.y = y

        #  (0, 0, sin(theta/2), cos(theta/2)).
        move_base_goal.target_pose.pose.orientation.z =  math.sin(self.yaw/2)
        move_base_goal.target_pose.pose.orientation.w =  math.cos(self.yaw/2)
        

        self.client.send_goal(move_base_goal)
        state = self.client.get_state()

        # finished_within_time = self.client.wait_for_result()

        prev_point = self.get_current_xy()
        time_start = time.time()

        distance_threshold = 0.2

        goal_diff = math.sqrt( (x-prev_point[0])**2 + (y-prev_point[1])**2 )

        TIMEOUT = 7 if goal_diff >=0.5 else 3

        print "\n[move_base] Sent MoveBase Goal :[ %s , %s , %s]. Will TIMEOUT in %s \n"%(x, y, self.yaw, TIMEOUT)

        diff = distance_threshold

        while  diff >= distance_threshold  and (state==GoalStatus.ACTIVE or state==GoalStatus.PENDING) :
            cur_time = time.time()
            curr_point = self.get_current_xy()
            state = self.client.get_state()
            if abs(cur_time - time_start) >= TIMEOUT :
                TIMEOUT = 3
                diff = math.sqrt( (curr_point[0]-prev_point[0])**2 + (curr_point[1]-prev_point[1])**2 )
                prev_point = curr_point
                time_start = time.time()

        if state==GoalStatus.SUCCEEDED or  state==GoalStatus.ABORTED:
            print "\n[move_base] Goal End. Status:%s\n"%(state)
            return state


        state = self.client.get_state()
        self.client.cancel_goal()
        print "\n[move_base] Goal Cancelled. TIMEOUT! Status: %s\n"%(state)

        return GoalStatus.SUCCEEDED


    def isValid(self):
        x,y = self.goal
        if ( x <= self.soft_xmax and x >= self.soft_xmin ) and ( y <= self.soft_ymax and y >= self.soft_ymin ) :
            return True

        return False
    

    def get_current_xy(self):
        listener = tf.TransformListener()
        while True:
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans[0],trans[1]
        

    def handle_goal(self, msg):

        print "\n[goal_server]: Recieved Goal (X,Y) -> ( %s,%s ) \n " %(msg.position.x, msg.position.y)

        self.goal = [msg.position.x, msg.position.y]

        self.yaw = 0
        self.get_orientation()
        self.marker_publish(2)

        valid = self.isValid()
        if valid == False:
            print "\n[goal_server]: Goal outside bounds \n"
            
        self.generate_new_goal(valid)

        
        print "\n[goal_server] New Safe Goal1 (X,Y): %s Yaw: %s \n"%(self.goal,self.yaw)

        if not valid:
            inter_x, inter_y = self.plan_to_goal()
            print "\n[goal_server] New Safe Goal2 (X,Y): %s Yaw: %s \n"%(self.goal,self.yaw)
        else:
            inter_x, inter_y = self.goal

        self.marker_publish(1)

        state = self.movebase_client(inter_x, inter_y)

        if state!=GoalStatus.SUCCEEDED:
            print "\n[move_base] Goal Aborted. Trying again \n"
            state = self.movebase_client(inter_x, inter_y)

        if state== GoalStatus.ABORTED:
            print "\n[goal_server] Goal Aborted Again. Aborting Attempt. "
            print(raw_input("[goal_server]: Teleop to desired goal. Enter (Y/N) to end TASK : ") )
            print "\n[goal_server] TASK COMPLETED \n"
            resp = Bool()
            resp.data = False
            self.response_pub.publish(resp)
            return

        if state== GoalStatus.SUCCEEDED:
            print "\n[move_base] Goal Reached \n"

        print "\n[goal_server] Starting Approach \n"

        self.approach_yaw()

        self.approach_goal()

        self.approach_yaw()

        # x,y = self.get_current_xy()
        # x,y = self.goal

        # self.movebase_client(x,y)

        print "\n[goal_server] End of Approach. Have some Candy!  \n"
        
        rospy.sleep(5)

        print "\n[goal_server] Starting Retreat  \n"

        self.retreat()

        print "\n[goal_server] End of Retreat  \n"

        
        print "\n[goal_server] TASK COMPLETED \n"
            
        resp = Bool()
        resp.data = True
        self.response_pub.publish(resp)

        return 

    
    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        self.vel_pub.publish(cmd)

    
    def generate_new_goal(self, valid):

        x,y = self.goal

        if not valid:
            
            if x > self.soft_xmax :
                x = self.soft_xmax
            elif x < self.soft_xmin :
                x = self.soft_xmin

            if y > self.soft_ymax :
                y = self.soft_ymax
            elif y < self.soft_ymin :
                y = self.soft_ymin
            
            self.goal = [ x, y ]
        
        else:

            center_x , center_y = self.center_arena

            x_check , y_check = (x - center_x) , (y - center_y)

            if x_check == 0 :
                x_check = 0.0001
            
            if y_check == 0:
                y_check = 0.0001
                

            else:
                angle = math.atan2(y_check,x_check)

                if (angle <= math.pi/4 and angle >= -math.pi/4):
                    self.goal = [self.soft_xmax, y]
                    self.yaw = 0

                elif (angle <= 3*math.pi/4 and angle >=math.pi/4):
                    self.goal = [x, self.soft_ymax]
                    self.yaw = math.pi/2

                elif (angle <= -math.pi/4 and angle >= -3*math.pi/4):
                    self.goal = [x, self.soft_ymin]
                    self.yaw = - math.pi/2

                else:
                    self.goal = [self.soft_xmin, y]
                    self.yaw =  math.pi











if __name__=="__main__":
    Seq1 = Sequence()

