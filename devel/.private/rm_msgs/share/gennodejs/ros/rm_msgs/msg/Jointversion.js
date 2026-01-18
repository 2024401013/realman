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

class Jointversion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_version = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_version')) {
        this.joint_version = initObj.joint_version
      }
      else {
        this.joint_version = new Array(7).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Jointversion
    // Check that the constant length array field [joint_version] has the right length
    if (obj.joint_version.length !== 7) {
      throw new Error('Unable to serialize array field joint_version - length must be 7')
    }
    // Serialize message field [joint_version]
    bufferOffset = _arraySerializer.string(obj.joint_version, buffer, bufferOffset, 7);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Jointversion
    let len;
    let data = new Jointversion(null);
    // Deserialize message field [joint_version]
    data.joint_version = _arrayDeserializer.string(buffer, bufferOffset, 7)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.joint_version.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Jointversion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34fe8048af36c99d06616a2279b77cd6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[7] joint_version
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Jointversion(null);
    if (msg.joint_version !== undefined) {
      resolved.joint_version = msg.joint_version;
    }
    else {
      resolved.joint_version = new Array(7).fill(0)
    }

    return resolved;
    }
};

module.exports = Jointversion;
