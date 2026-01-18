// Auto-generated. Do not edit!

// (in-package rm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Softwarebuildinfo = require('./Softwarebuildinfo.js');

//-----------------------------------------------------------

class Arm_Software_Version {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.product_version = null;
      this.algorithm_info = null;
      this.ctrl_info = null;
      this.dynamic_info = null;
      this.plan_info = null;
      this.controller_version = null;
      this.com_info = null;
      this.program_info = null;
    }
    else {
      if (initObj.hasOwnProperty('product_version')) {
        this.product_version = initObj.product_version
      }
      else {
        this.product_version = '';
      }
      if (initObj.hasOwnProperty('algorithm_info')) {
        this.algorithm_info = initObj.algorithm_info
      }
      else {
        this.algorithm_info = '';
      }
      if (initObj.hasOwnProperty('ctrl_info')) {
        this.ctrl_info = initObj.ctrl_info
      }
      else {
        this.ctrl_info = new Softwarebuildinfo();
      }
      if (initObj.hasOwnProperty('dynamic_info')) {
        this.dynamic_info = initObj.dynamic_info
      }
      else {
        this.dynamic_info = '';
      }
      if (initObj.hasOwnProperty('plan_info')) {
        this.plan_info = initObj.plan_info
      }
      else {
        this.plan_info = new Softwarebuildinfo();
      }
      if (initObj.hasOwnProperty('controller_version')) {
        this.controller_version = initObj.controller_version
      }
      else {
        this.controller_version = '';
      }
      if (initObj.hasOwnProperty('com_info')) {
        this.com_info = initObj.com_info
      }
      else {
        this.com_info = new Softwarebuildinfo();
      }
      if (initObj.hasOwnProperty('program_info')) {
        this.program_info = initObj.program_info
      }
      else {
        this.program_info = new Softwarebuildinfo();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Arm_Software_Version
    // Serialize message field [product_version]
    bufferOffset = _serializer.string(obj.product_version, buffer, bufferOffset);
    // Serialize message field [algorithm_info]
    bufferOffset = _serializer.string(obj.algorithm_info, buffer, bufferOffset);
    // Serialize message field [ctrl_info]
    bufferOffset = Softwarebuildinfo.serialize(obj.ctrl_info, buffer, bufferOffset);
    // Serialize message field [dynamic_info]
    bufferOffset = _serializer.string(obj.dynamic_info, buffer, bufferOffset);
    // Serialize message field [plan_info]
    bufferOffset = Softwarebuildinfo.serialize(obj.plan_info, buffer, bufferOffset);
    // Serialize message field [controller_version]
    bufferOffset = _serializer.string(obj.controller_version, buffer, bufferOffset);
    // Serialize message field [com_info]
    bufferOffset = Softwarebuildinfo.serialize(obj.com_info, buffer, bufferOffset);
    // Serialize message field [program_info]
    bufferOffset = Softwarebuildinfo.serialize(obj.program_info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Arm_Software_Version
    let len;
    let data = new Arm_Software_Version(null);
    // Deserialize message field [product_version]
    data.product_version = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [algorithm_info]
    data.algorithm_info = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ctrl_info]
    data.ctrl_info = Softwarebuildinfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [dynamic_info]
    data.dynamic_info = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [plan_info]
    data.plan_info = Softwarebuildinfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [controller_version]
    data.controller_version = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [com_info]
    data.com_info = Softwarebuildinfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [program_info]
    data.program_info = Softwarebuildinfo.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.product_version);
    length += _getByteLength(object.algorithm_info);
    length += Softwarebuildinfo.getMessageSize(object.ctrl_info);
    length += _getByteLength(object.dynamic_info);
    length += Softwarebuildinfo.getMessageSize(object.plan_info);
    length += _getByteLength(object.controller_version);
    length += Softwarebuildinfo.getMessageSize(object.com_info);
    length += Softwarebuildinfo.getMessageSize(object.program_info);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Arm_Software_Version';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f90eb6551bbde202ac1756e5a3ee2f6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string product_version
    string algorithm_info
    Softwarebuildinfo ctrl_info
    string dynamic_info #3
    Softwarebuildinfo plan_info #3
    string controller_version #4
    Softwarebuildinfo com_info #4
    Softwarebuildinfo program_info #4
    ================================================================================
    MSG: rm_msgs/Softwarebuildinfo
    string build_time
    string version
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Arm_Software_Version(null);
    if (msg.product_version !== undefined) {
      resolved.product_version = msg.product_version;
    }
    else {
      resolved.product_version = ''
    }

    if (msg.algorithm_info !== undefined) {
      resolved.algorithm_info = msg.algorithm_info;
    }
    else {
      resolved.algorithm_info = ''
    }

    if (msg.ctrl_info !== undefined) {
      resolved.ctrl_info = Softwarebuildinfo.Resolve(msg.ctrl_info)
    }
    else {
      resolved.ctrl_info = new Softwarebuildinfo()
    }

    if (msg.dynamic_info !== undefined) {
      resolved.dynamic_info = msg.dynamic_info;
    }
    else {
      resolved.dynamic_info = ''
    }

    if (msg.plan_info !== undefined) {
      resolved.plan_info = Softwarebuildinfo.Resolve(msg.plan_info)
    }
    else {
      resolved.plan_info = new Softwarebuildinfo()
    }

    if (msg.controller_version !== undefined) {
      resolved.controller_version = msg.controller_version;
    }
    else {
      resolved.controller_version = ''
    }

    if (msg.com_info !== undefined) {
      resolved.com_info = Softwarebuildinfo.Resolve(msg.com_info)
    }
    else {
      resolved.com_info = new Softwarebuildinfo()
    }

    if (msg.program_info !== undefined) {
      resolved.program_info = Softwarebuildinfo.Resolve(msg.program_info)
    }
    else {
      resolved.program_info = new Softwarebuildinfo()
    }

    return resolved;
    }
};

module.exports = Arm_Software_Version;
