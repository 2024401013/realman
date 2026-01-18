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

class Softwarebuildinfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.build_time = null;
      this.version = null;
    }
    else {
      if (initObj.hasOwnProperty('build_time')) {
        this.build_time = initObj.build_time
      }
      else {
        this.build_time = '';
      }
      if (initObj.hasOwnProperty('version')) {
        this.version = initObj.version
      }
      else {
        this.version = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Softwarebuildinfo
    // Serialize message field [build_time]
    bufferOffset = _serializer.string(obj.build_time, buffer, bufferOffset);
    // Serialize message field [version]
    bufferOffset = _serializer.string(obj.version, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Softwarebuildinfo
    let len;
    let data = new Softwarebuildinfo(null);
    // Deserialize message field [build_time]
    data.build_time = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [version]
    data.version = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.build_time);
    length += _getByteLength(object.version);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Softwarebuildinfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7075122606129dab433ee6c359a4c404';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string build_time
    string version
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Softwarebuildinfo(null);
    if (msg.build_time !== undefined) {
      resolved.build_time = msg.build_time;
    }
    else {
      resolved.build_time = ''
    }

    if (msg.version !== undefined) {
      resolved.version = msg.version;
    }
    else {
      resolved.version = ''
    }

    return resolved;
    }
};

module.exports = Softwarebuildinfo;
