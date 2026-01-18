// Auto-generated. Do not edit!

// (in-package rm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Modbustcpmasterinfo = require('./Modbustcpmasterinfo.js');

//-----------------------------------------------------------

class Modbustcpmasterlist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.master_list = null;
    }
    else {
      if (initObj.hasOwnProperty('master_list')) {
        this.master_list = initObj.master_list
      }
      else {
        this.master_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Modbustcpmasterlist
    // Serialize message field [master_list]
    // Serialize the length for message field [master_list]
    bufferOffset = _serializer.uint32(obj.master_list.length, buffer, bufferOffset);
    obj.master_list.forEach((val) => {
      bufferOffset = Modbustcpmasterinfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Modbustcpmasterlist
    let len;
    let data = new Modbustcpmasterlist(null);
    // Deserialize message field [master_list]
    // Deserialize array length for message field [master_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.master_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.master_list[i] = Modbustcpmasterinfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.master_list.forEach((val) => {
      length += Modbustcpmasterinfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Modbustcpmasterlist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b8086f234a37bbe3de0d031d6dca80f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Modbustcpmasterinfo[] master_list   # 返回符合的TCP主站列表
    
    ================================================================================
    MSG: rm_msgs/Modbustcpmasterinfo
    string master_name # Modbus 主站名称，最大长度15个字符
    string ip          # TCP主站 IP 地址
    int32 port         # TCP主站端口号	
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Modbustcpmasterlist(null);
    if (msg.master_list !== undefined) {
      resolved.master_list = new Array(msg.master_list.length);
      for (let i = 0; i < resolved.master_list.length; ++i) {
        resolved.master_list[i] = Modbustcpmasterinfo.Resolve(msg.master_list[i]);
      }
    }
    else {
      resolved.master_list = []
    }

    return resolved;
    }
};

module.exports = Modbustcpmasterlist;
