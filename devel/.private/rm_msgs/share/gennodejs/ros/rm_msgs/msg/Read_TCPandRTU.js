// Auto-generated. Do not edit!

// (in-package rm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Read_TCPandRTU {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.address = null;
      this.num = null;
      this.ip = null;
      this.port = null;
      this.master_name = null;
      this.device = null;
      this.type = null;
    }
    else {
      if (initObj.hasOwnProperty('address')) {
        this.address = initObj.address
      }
      else {
        this.address = 0;
      }
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = 0;
      }
      if (initObj.hasOwnProperty('ip')) {
        this.ip = initObj.ip
      }
      else {
        this.ip = '';
      }
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
      if (initObj.hasOwnProperty('master_name')) {
        this.master_name = initObj.master_name
      }
      else {
        this.master_name = '';
      }
      if (initObj.hasOwnProperty('device')) {
        this.device = initObj.device
      }
      else {
        this.device = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Read_TCPandRTU
    // Serialize message field [address]
    bufferOffset = _serializer.int32(obj.address, buffer, bufferOffset);
    // Serialize message field [num]
    bufferOffset = _serializer.int32(obj.num, buffer, bufferOffset);
    // Serialize message field [ip]
    bufferOffset = _serializer.string(obj.ip, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.int32(obj.port, buffer, bufferOffset);
    // Serialize message field [master_name]
    bufferOffset = _serializer.string(obj.master_name, buffer, bufferOffset);
    // Serialize message field [device]
    bufferOffset = _serializer.int32(obj.device, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Read_TCPandRTU
    let len;
    let data = new Read_TCPandRTU(null);
    // Deserialize message field [address]
    data.address = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num]
    data.num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ip]
    data.ip = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [master_name]
    data.master_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [device]
    data.device = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.ip);
    length += _getByteLength(object.master_name);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Read_TCPandRTU';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90a88b29976390e1b70c71f43af18e9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 address
    int32 num
    string ip
    int32 port
    string master_name
    int32 device #外设设备地址。
    int32 type #0-控制器端modbus主机；1-工具端modbus主机。3-TCP模式
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Read_TCPandRTU(null);
    if (msg.address !== undefined) {
      resolved.address = msg.address;
    }
    else {
      resolved.address = 0
    }

    if (msg.num !== undefined) {
      resolved.num = msg.num;
    }
    else {
      resolved.num = 0
    }

    if (msg.ip !== undefined) {
      resolved.ip = msg.ip;
    }
    else {
      resolved.ip = ''
    }

    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.master_name !== undefined) {
      resolved.master_name = msg.master_name;
    }
    else {
      resolved.master_name = ''
    }

    if (msg.device !== undefined) {
      resolved.device = msg.device;
    }
    else {
      resolved.device = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    return resolved;
    }
};

module.exports = Read_TCPandRTU;
