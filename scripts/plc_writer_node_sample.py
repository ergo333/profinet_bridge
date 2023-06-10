#!/usr/bin/env python3

import rospy
import struct 
import time
from profinet_bridge.msg import ProfinetWriteMsg
from profinet_bridge.srv import DataWriteSrv

def update_db_bool_client(var_name, var_data):

    rospy.wait_for_service('profinet_bridge/write')
    try:
        plc_service = rospy.ServiceProxy('profinet_bridge/write', DataWriteSrv)
        response    = plc_service(var_name, var_data)
        return response.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    rospy.init_node('profinet_bridge_sample_node', anonymous=True)

    data_pub = rospy.Publisher('profinet_bridge/write', ProfinetWriteMsg, queue_size=1)

    '''
        Writing a sample boolean variable
    '''
    # while not rospy.is_shutdown():
    #     test_bool = [0, 1]
    #     message = ProfinetWriteMsg()
    #     message.variable_name = 'testBool'
    #     message.data = test_bool
    #     data_pub.publish(message)
    #     time.sleep(0.2)


    '''
        Writing a sample int variable
    '''
    # while not rospy.is_shutdown():
    #     test_int = [0, 100]
    #     message = ProfinetWriteMsg()
    #     message.variable_name = 'testInt'
    #     message.data = test_int
    #     data_pub.publish(message)
    #     time.sleep(0.2)


    '''
        Writing a sample float variable
    # '''
    # while not rospy.is_shutdown():
    #     test_real = [64, 58, 1, 102]
    #     message = ProfinetWriteMsg()
    #     message.variable_name = 'testReal'
    #     message.data = test_real
    #     data_pub.publish(message)
    #     time.sleep(0.2)


    '''
        Calling service for boolean function
    '''
    # res = update_db_bool_client('testBool', [1, 0])
    # print(res)

    '''
        Calling service for real function
    '''
    res = update_db_bool_client('testReal', [4, 5, 6, 7])
    print(res)

if __name__ == "__main__":
    main()