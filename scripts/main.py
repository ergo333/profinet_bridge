#!/usr/bin/env python3

import rospy
import time
import struct
from plc_communication import PLCCommunicator
from profinet_bridge.msg import ProfinetWriteMsg, ProfinetReadMsg, ProfinetDataMsg
from profinet_bridge.srv import DataReadSrv, DataReadSrvResponse, DataWriteSrv, DataWriteSrvResponse

'''
    Function to find type of a variable
'''
def getType(var_name: str, database_variables):
    
    for variable in database_variables:
        if var_name == list(variable.keys())[0]:
            return variable[var_name]['type']
    return None

def getTypeDimension(var_type: str, database_types):

    if var_type is None:
        return None
    
    return database_types[var_type]


def db_write(variables, types, PLC, variable_name, variable_value):

    for var in variables:
        
        var_name = [name for name in var.keys()]
        if var_name[0] == variable_name:

            variable_type   = var[var_name[0]]['type']
            variable_offset = var[var_name[0]]['offset']
            variable_dim    = types[variable_type]

            PLC.write(variable_offset, variable_value) 
            return True
    return False



'''
Callback that listens from the topic the data and writes to the PLC
args is a tuple: (variables, types, plc, db_number)
'''
def write_data_cb(profinet_msg, args):
    variables = args[0]
    types     = args[1]
    plc       = args[2]

    variable_name = profinet_msg.variable_name
    variable_value = profinet_msg.data

    print(f"Variable name: {variable_name} , Variable value: {variable_value}")
    
    data_packed = [struct.pack('b', element) for element in variable_value]
    data_ready = bytearray()
    for data in data_packed:
        data_ready.extend(data)
    

    if not db_write(variables, types, plc, variable_name, data_ready):
        print(f"[PROFINET_BRIDGE] Variable {variable_name} not found!")
    else:
        print(f"[PROFINET_BRIDGE] Variable {variable_name} updated!")



'''
Returns true if there is a correspondence between types and variables
'''
def check_variable_types(types, variables):

    for var in variables:
        var_name = [name for name in var.keys()]

        if var[var_name[0]]['type'] not in types:
            return False

    return True

'''
    ROS Service for reading data from database
'''
def read_service_cb(req, database_variables, database_types, PLC):

    response = DataReadSrvResponse()
    
    for var in database_variables:
        
        var_name = [name for name in var.keys()]
        
        if var_name[0] == req.variable_name:
            
            var_type = getType(var_name[0], database_variables)
            
            if var_type is None:
                response.data = []
            else:
                data_plc = PLC.read(var[var_name[0]]['offset'], getTypeDimension(var_type, database_types))
                response.data = [struct.unpack('b', bytes([element]))[0] for element in list(data_plc)]
                
           
    return response

'''
    ROS Service for writing data to database
'''
def write_service_cb(req, variables, types, plc):

    variable_name = req.variable_name
    variable_value = req.data

    print(f"Variable name: {variable_name} , Variable value: {variable_value}")
    
    data_packed = [struct.pack('b', element) for element in variable_value]
    data_ready = bytearray()
    for data in data_packed:
        data_ready.extend(data)
    
    response = DataWriteSrvResponse()
    response.result = db_write(variables, types, plc, variable_name, data_ready)
    return response


'''
    Main function that activates rosservice / publishers and subscribers
'''
def main():
    rospy.init_node('profinet_bridge', anonymous=True)
    
    '''
        Reading parameters from launchfile
    '''
    PLC_ip   = rospy.get_param("~PLC_ip")
    PLC_rack = rospy.get_param("~PLC_rack")
    PLC_slot = rospy.get_param("~PLC_slot")
    PLC_port = rospy.get_param("~PLC_port")

    data_write_topic   = rospy.get_param("~data_write_topic")
    data_read_topic    = rospy.get_param("~data_read_topic")
    data_write_service = rospy.get_param("~data_write_service")
    data_read_service  = rospy.get_param("~data_read_service")
    frequency          = rospy.get_param("~frequency")

    '''
        Loading database configuration
    '''
    database_number    = rospy.get_param("~database/number")
    database_types     = rospy.get_param("~database/types")
    database_variables = rospy.get_param("~database/variables")

    print(f"[PROFINET_BRIDGE] Database number: {database_number}")
    print(f"[PROFINET_BRIDGE] Database types: {database_types}")
    print(f"[PROFINET_BRIDGE] Database variables: {database_variables}")


    print(f"[PROFINET_BRIDGE] Connecting to PLC ...")
    PLC = PLCCommunicator(PLC_ip, PLC_rack, PLC_slot, PLC_port, database_number)
    print(f"[PROFINET_BRIDGE] Connected to IP: {PLC_ip}, rack: {PLC_rack}, slot: {PLC_slot}, port: {PLC_port}")


    if check_variable_types(database_types.keys(), database_variables):
        print("[PROFINET_BRIDGE] Types and variables match!")
    else:
        print("[PROFINET_BRIDGE] Mismatch between variables and defined types")
        raise Exception("Cannot run profinet_bridge")
    

    # Creating data write Subscriber
    rospy.Subscriber(data_write_topic, ProfinetWriteMsg, write_data_cb, (database_variables, database_types, PLC))
    
    # Creating data read Publisher
    data_pub = rospy.Publisher(data_read_topic, ProfinetReadMsg, queue_size=1)

    rate = rospy.Rate(frequency)


    # Initialization of Read and Write data services
    read_service  = rospy.Service(data_read_service, DataReadSrv, lambda msg : read_service_cb(msg, database_variables, database_types, PLC))
    write_service = rospy.Service(data_write_service, DataWriteSrv, lambda msg : write_service_cb(msg, database_variables, database_types, PLC)) 

    while not rospy.is_shutdown():

        data_msgs = []
        for var in database_variables:
            data_msg = ProfinetDataMsg()
            var_name = [name for name in var.keys()]
            data_msg.name = var_name[0]
            data_msg.type = var[var_name[0]]['type']

            data_plc = PLC.read(var[var_name[0]]['offset'], database_types[data_msg.type])            
            data_msg.data = [struct.unpack('b', bytes([element]))[0] for element in list(data_plc)]
            
            
            data_msgs.append(data_msg)
            time.sleep(float(1)/float(frequency))

        message = ProfinetReadMsg()
        message.variables = data_msgs

        data_pub.publish(message)

        rate.sleep()

    PLC.close_connection()

if __name__ == "__main__":
    main()