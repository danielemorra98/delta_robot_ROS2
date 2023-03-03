#! /usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3 
from sensor_msgs.msg import JointState 

class DeltaRobot:
    def __init__(self):
        self.e  =  5.0 # End-effector radius
        self.f  =  50.0 # Base platform radius
        self.re = 100.0 # Distrance from elbow to the wrist
        self.rf =  50.0 # Distance from motor shaft to elbow

        self.base_origin_x = 0.0
        self.base_origin_y = 0.0
        self.base_origin_z = 0.0
        self.bias_angle_base = -np.pi/2

        # auxiliary variables
        self.E = [0,0,0]
        self.F = [0,0,0]
        self.G = [0,0,0]

        self.current_theta = [0,0,0]

        self.plot_boolean = False
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.bicep_line = [0,0,0]
        self.forearm_line = [0,0,0]
        self.ee_triangle_desired = None


    def inverse(self, x, y, z):
        self.initialize_auxiliary_parameters(x, y, z)

        theta = [0,0,0]
        for i in range(3):
            delta = self.E[i]**2 + self.F[i]**2 - self.G[i]**2
            if delta < 0:
                raise Exception(f"Delta {i} less than 0!")

            t_half_angle = (- self.F[i] - np.sqrt(delta))/(self.G[i]-self.E[i])

            theta[i] = 2 * np.arctan(t_half_angle) * 180 / np.pi

        return theta 

    def initialize_auxiliary_parameters(self, x, y, z):
        a = 0.5*self.f - self.e
        b = 0.5*np.sqrt(3)*(self.e-0.5*self.f)
        c = 0.5*(self.e-0.5*self.f)

        self.E[0] = 2*self.rf*(y+a)
        self.F[0] = 2*self.rf*z
        self.G[0] = x**2 + y**2 + z**2 + a**2 + self.rf**2 + 2*y*a - self.re**2

        self.E[1] = -self.rf*(np.sqrt(3)*(x+b)+y+c)
        self.F[1] = 2*self.rf*z
        self.G[1] = x**2 + y**2 + z**2 + b**2 + c**2 + self.rf**2 + 2*(x*b + y*c) - self.re**2

        self.E[2] = self.rf*(np.sqrt(3)*(x-b)-y-c)
        self.F[2] = 2*self.rf*z
        self.G[2] = x**2 + y**2 + z**2 + b**2 + c**2 + self.rf**2 + 2*(-x*b + y*c) - self.re**2

    # Implementaton based on Wikipedia Trilateration article.                              
    def trilaterate(self, P1,P2,P3,radius):                      
        temp1 = P2-P1                                        
        e_x = temp1/np.linalg.norm(temp1)                              
        temp2 = P3-P1                                        
        i = np.dot(e_x,temp2)                                   
        temp3 = temp2 - i*e_x                                
        e_y = temp3/np.linalg.norm(temp3)                              
        e_z = np.cross(e_x,e_y)                                 
        d = np.linalg.norm(P2-P1)                                      
        j = np.dot(e_y,temp2)                                   
        x = (radius*radius - radius*radius + d*d) / (2*d)                    
        y = (radius*radius - radius*radius -2*i*x + i*i + j*j) / (2*j)       
        temp4 = radius*radius - x*x - y*y                            
        if temp4<0:                                          
            raise Exception("The three spheres do not intersect!")
        z = np.sqrt(temp4)                                      
        p_12_a = P1 + x*e_x + y*e_y + z*e_z                  
        p_12_b = P1 + x*e_x + y*e_y - z*e_z                  
        return p_12_a,p_12_b   

    # Define a function to compute the forward kinematics of the delta robot
    def forward(self, theta1, theta2, theta3):
        """
        Computes the forward kinematics of a delta robot given the
        joint angles of its legs.
        """
        base_origin = np.asarray([self.base_origin_x, self.base_origin_y, self.base_origin_z])

        motor1 = base_origin + np.asarray([self.f/2*np.cos(self.bias_angle_base), self.f/2*np.sin(self.bias_angle_base), 0])
        motor2 = base_origin + np.asarray([self.f/2*np.cos(2/3*np.pi+self.bias_angle_base), self.f/2*np.sin(2/3*np.pi+self.bias_angle_base), 0])
        motor3 = base_origin + np.asarray([self.f/2*np.cos(4/3*np.pi+self.bias_angle_base), self.f/2*np.sin(4/3*np.pi+self.bias_angle_base), 0])
        # print("computation - x_motor", motor1, motor2, motor3)

        elbow1 = motor1 + np.asarray([self.rf*np.cos(-np.radians(theta1))*np.cos(self.bias_angle_base), self.rf*np.cos(-np.radians(theta1))*np.sin(self.bias_angle_base), self.rf*np.sin(-np.radians(theta1))])
        elbow2 = motor2 + np.asarray([self.rf*np.cos(-np.radians(theta2))*np.cos(2/3*np.pi+self.bias_angle_base), self.rf*np.cos(-np.radians(theta2))*np.sin(2/3*np.pi+self.bias_angle_base), self.rf*np.sin(-np.radians(theta2))])
        elbow3 = motor3 + np.asarray([self.rf*np.cos(-np.radians(theta3))*np.cos(4/3*np.pi+self.bias_angle_base), self.rf*np.cos(-np.radians(theta3))*np.sin(4/3*np.pi+self.bias_angle_base), self.rf*np.sin(-np.radians(theta3))])
        # print("computation - x_elbow", elbow1, elbow2, elbow3)

        center1 = elbow1 - np.asarray([self.e*np.cos(self.bias_angle_base), self.e*np.sin(self.bias_angle_base), 0])
        center2 = elbow2 - np.asarray([self.e*np.cos(2/3*np.pi+self.bias_angle_base), self.e*np.sin(2/3*np.pi+self.bias_angle_base), 0])
        center3 = elbow3 - np.asarray([self.e*np.cos(4/3*np.pi+self.bias_angle_base), self.e*np.sin(4/3*np.pi+self.bias_angle_base), 0])

        point1, point2 = self.trilaterate(center1, center2, center3, self.re)

        if point1[2] < 0:
            return point1
        if point2[2] < 0:
            return point2

        return None

    def update_current_position(self, position_msg):
        self.current_theta[0] = position_msg[0] * 180 /np.pi
        self.current_theta[1] = position_msg[1] * 180 /np.pi
        self.current_theta[2] = position_msg[2] * 180 /np.pi

        if self.plot_boolean:
            self.update_plot_with_feedback()

    
    def plot(self, theta):
        # Plot the base platform
        plt.ion()
        base_x = np.array([-np.sqrt(3)/2*self.f, np.sqrt(3)/2*self.f, 0])
        base_y = np.array([-0.5*self.f, -0.5*self.f, self.f])
        base_z = np.array([0, 0, 0])

        self.base_triangle = self.ax.plot_trisurf(base_x, base_y, base_z, linewidth=0.2, color='gray', antialiased=True)

        # Plot the end-effector platform
        
        x, y, z = self.forward(theta[0], theta[1], theta[2])
        end_effector_x = np.array([-np.sqrt(3)/2*self.e+x, np.sqrt(3)/2*self.e+x, x])
        end_effector_y = np.array([0.5*self.e+y, 0.5*self.e+y, -self.e+y])
        end_effector_z = np.array([z, z, z])

        self.ee_triangle = self.ax.plot_trisurf(end_effector_x, end_effector_y, end_effector_z, linewidth=0.2, color='blue', antialiased=True)

        for i in range(3):
            angle_base = self.bias_angle_base + 2/3*np.pi*i

            x_motor = self.base_origin_x + self.f/2*np.cos(angle_base)
            y_motor = self.base_origin_y + self.f/2*np.sin(angle_base)
            z_motor = self.base_origin_z
            motor = [x_motor, y_motor, z_motor]

            x_elbow = x_motor + self.rf*np.cos(-np.radians(theta[i]))*np.cos(angle_base)
            y_elbow = y_motor + self.rf*np.cos(-np.radians(theta[i]))*np.sin(angle_base)
            z_elbow = z_motor + self.rf*np.sin(-np.radians(theta[i]))
            elbow = [x_elbow, y_elbow, z_elbow]

            self.bicep_line[i], = self.ax.plot([motor[0], elbow[0]], [motor[1], elbow[1]], [motor[2], elbow[2]], color='green')

            angle_ee = angle_base
            x_wrist = x + self.e*np.cos(angle_ee)
            y_wrist = y + self.e*np.sin(angle_ee)
            z_wrist = z
            wrist = [x_wrist, y_wrist, z_wrist]

            self.forearm_line[i], = self.ax.plot([wrist[0], elbow[0]], [wrist[1], elbow[1]], [wrist[2], elbow[2]], color='red')

        # Set axis limits and labels
        self.ax.set_xlim(-70, 70)
        self.ax.set_ylim(-70, 70)
        self.ax.set_zlim(-150, 20)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        
        self.fig.canvas.draw()
        plt.pause(0.001)
        # self.fig.canvas.flush_events()


    def update_plot_with_feedback(self):
        x, y, z = self.forward(self.current_theta[0], self.current_theta[1], self.current_theta[2])
        end_effector_x = np.array([-np.sqrt(3)/2*self.e+x, np.sqrt(3)/2*self.e+x, x])
        end_effector_y = np.array([0.5*self.e+y, 0.5*self.e+y, -self.e+y])
        end_effector_z = np.array([z, z, z])

        self.ee_triangle.set_verts([list(zip(end_effector_x, end_effector_y, end_effector_z))])

        for i in range(3):
            angle_base = self.bias_angle_base + 2/3*np.pi*i

            x_motor = self.base_origin_x + self.f/2*np.cos(angle_base)
            y_motor = self.base_origin_y + self.f/2*np.sin(angle_base)
            z_motor = self.base_origin_z
            motor = [x_motor, y_motor, z_motor]

            x_elbow = x_motor + self.rf*np.cos(-np.radians(self.current_theta[i]))*np.cos(angle_base)
            y_elbow = y_motor + self.rf*np.cos(-np.radians(self.current_theta[i]))*np.sin(angle_base)
            z_elbow = z_motor + self.rf*np.sin(-np.radians(self.current_theta[i]))
            elbow = [x_elbow, y_elbow, z_elbow]

            self.bicep_line[i].set_data([motor[0], elbow[0]], [motor[1], elbow[1]])
            self.bicep_line[i].set_3d_properties([motor[2], elbow[2]])

            angle_ee = angle_base
            x_wrist = x + self.e*np.cos(angle_ee)
            y_wrist = y + self.e*np.sin(angle_ee)
            z_wrist = z
            wrist = [x_wrist, y_wrist, z_wrist]

            self.forearm_line[i].set_data([wrist[0], elbow[0]], [wrist[1], elbow[1]])
            self.forearm_line[i].set_3d_properties([wrist[2], elbow[2]])
        
        self.fig.canvas.draw()
        plt.pause(0.001)


    def update_with_desired_plot(self, theta):
        x, y, z = self.forward(theta[0], theta[1], theta[2])
        end_effector_x = np.array([-np.sqrt(3)/2*self.e+x, np.sqrt(3)/2*self.e+x, x])
        end_effector_y = np.array([0.5*self.e+y, 0.5*self.e+y, -self.e+y])
        end_effector_z = np.array([z, z, z])

        if (self.ee_triangle_desired == None):
            self.ee_triangle_desired = self.ax.plot_trisurf(end_effector_x, end_effector_y, end_effector_z, linewidth=0.2, color='cyan', antialiased=True)
        else:
            self.ee_triangle_desired.set_verts([list(zip(end_effector_x, end_effector_y, end_effector_z))])


        self.fig.canvas.draw()
        plt.pause(0.001)    

class KinematicsPublisher(Node):

    def __init__(self):
        super().__init__('delta_robot')
        self.my_robot = DeltaRobot()
        self.publisher = self.create_publisher(JointState, 'delta_robot/desired_joint_state', 10)
        self.subscription = self.create_subscription(
            Vector3, 
            'delta_robot/desired_position',
            self.desired_position_callback,
            10)
        self.subscription = self.create_subscription(
            JointState, 
            'delta_robot/joint_states',
            self.joint_states_callback,
            10)
        self.bias_simulation_angle = 0
        self.name_actuated_joint = ["motor1", "motor2", "motor3"]

    def desired_position_callback(self, msg):
        theta = self.my_robot.inverse(msg.x, msg.y, msg.z)
        if self.my_robot.plot_boolean:
            self.my_robot.update_with_desired_plot(theta)
        
        theta_msg = JointState()
        theta_msg.name.append(self.name_actuated_joint[0])
        theta_msg.position.append(np.radians(theta[0])-self.bias_simulation_angle)
        theta_msg.name.append(self.name_actuated_joint[1])
        theta_msg.position.append(np.radians(theta[1])-self.bias_simulation_angle)
        theta_msg.name.append(self.name_actuated_joint[2])
        theta_msg.position.append(np.radians(theta[2])-self.bias_simulation_angle)
        self.publisher.publish(theta_msg)

    def joint_states_callback(self, msg):
        theta = np.zeros(len(self.name_actuated_joint))
        j = 0
        for name in msg.name:
            for i in range(len(self.name_actuated_joint)):
                if name==self.name_actuated_joint[i]:
                    theta[i] = msg.position[j] + self.bias_simulation_angle
            j = j+1
        self.my_robot.update_current_position(theta)


def main():
    rclpy.init()
    node = KinematicsPublisher()
    node.my_robot.plot_boolean = True
    node.my_robot.plot(theta=[90,90,90])
    rclpy.spin(node)

    plt.show(block=False)

if __name__ == '__main__':
    main()