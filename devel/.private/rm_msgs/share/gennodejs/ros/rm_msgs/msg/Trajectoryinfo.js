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

class Trajectoryinfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.create_time = null;
      this.point_num = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('create_time')) {
        this.create_time = initObj.create_time
      }
      else {
        this.create_time = '';
      }
      if (initObj.hasOwnProperty('point_num')) {
        this.point_num = initObj.point_num
      }
      else {
        this.point_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Trajectoryinfo
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [create_time]
    bufferOffset = _serializer.string(obj.create_time, buffer, bufferOffset);
    // Serialize message field [point_num]
    bufferOffset = _serializer.int32(obj.point_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Trajectoryinfo
    let len;
    let data = new Trajectoryinfo(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [create_time]
    data.create_time = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [point_num]
    data.point_num = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.create_time);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Trajectoryinfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9fb2008a0ef07a81b5bc4d72b577068';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name          #轨迹名称	
    string create_time   #创建时间
    int32 point_num      #轨迹点数量
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Trajectoryinfo(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.create_time !== undefined) {
      resolved.create_time = msg.create_time;
    }
    else {
      resolved.create_time = ''
    }

    if (msg.point_num !== undefined) {
      resolved.point_num = msg.point_num;
    }
    else {
      resolved.point_num = 0
    }

    return resolved;
    }
};

module.exports = Trajectoryinfo;
