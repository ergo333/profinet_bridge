import snap7
from threading import Lock

class PLCCommunicator:

    '''
    Constructor function:
        takes as input the IP of the PLC, the rack number, the slot number and the tcpport
    '''
    def __init__(self, ip: int, rack: int, slot: int, port: int, db_number: int):
        try:
            self._client = snap7.client.Client()
            self._client.connect(ip, rack, slot, port)
            self._db_number = db_number
            self._mutex = Lock()
            assert(self._client.get_connected()), "Connection to PLC refused. Cannot instantiate PLCCommunicator"
        except:
            raise Exception("Connection to PLC refused. Cannot instantiate PLCCommunicator")

    '''
    Read function:
        input: 
            db_number - database number
            start     - starting byte index from where is start to read from
            size      - amount of bytes to be read
        return: 
            byte data 
    '''

    def read(self, start: int, size: int):
        self._mutex.acquire()
        result =  self._client.db_read(self._db_number, start, size)
        self._mutex.release()
        return result

    '''
    Write function:
        input:
            db_number – number of DB to be write
            start – byte index from where start to write to
            size – amount of bytes to write
            data – buffer to be write
        output:
            s7 error code
    '''
    def write(self, start: int, data: bytearray):
        self._mutex.acquire()
        self._client.db_write(self._db_number, start, data)   
        self._mutex.release()

    def close_connection(self):
        self._client.destroy()
    
