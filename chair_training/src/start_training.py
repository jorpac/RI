#!/usr/bin/env python

'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Moded by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import gym
import time
import numpy as np
import random
import time
import qlearn
from gym import wrappers
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg
# from live_plot import LivePlot

# import our training environment
import chair_env

def __numpy_to_string(A):
    return str(round(np.sum(A), 3))

if __name__ == '__main__':
    
    rospy.init_node('chair_gym', anonymous=True, log_level=rospy.DEBUG)

    # Create the Gym environment
    env = gym.make('Chair-v0')
    rospy.logdebug ( "Gym environment done")
    reward_pub = rospy.Publisher('/chair/reward', Float64, queue_size=1)
    episode_reward_pub = rospy.Publisher('/chair/episode_reward', Float64, queue_size=1)

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('chair_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")
    # Plot
    #plotter = LivePlot(outdir)


    last_time_steps = np.ndarray(0)
    action_size = 4
    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/alpha")
    Epsilon = rospy.get_param("/epsilon")
    Gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(action_size),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.logdebug ("STARTING Episode #"+str(x))
        
        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount
        
        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        state = env.reset()
        print('reset done')
        state = __numpy_to_string(state)
        
        # Show on screen the actual situation of the robot
        #env.render()
        
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):

            rospy.logdebug("###################### Start Step...["+str(i)+"]")

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.loginfo("Action to Perform >> "+str(action))
            
            nextState, reward, collision, done = env.step(action)
            rospy.logdebug("END Step...")
            rospy.loginfo("Reward ==> " + str(reward))
            nextState = __numpy_to_string(nextState)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            # Make the algorithm learn based on the results
            qlearn.learn(state, action, reward, nextState)

            # We publish the cumulated reward
            cumulated_reward_msg.data = cumulated_reward
            reward_pub.publish(cumulated_reward_msg)

            if not(collision) or not(done):
                state = nextState
            else:
                rospy.logdebug ("DONE")
                last_time_steps = np.append(last_time_steps, [int(i + 1)])
                break

            rospy.logdebug("###################### END Step...["+str(i)+"]")

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = cumulated_reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.logwarn( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))
        #plotter.plot(env)
    
    rospy.logdebug ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    rospy.logdebug("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.logdebug("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()
