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

class Modbusreaddata {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.read_data = null;
    }
    else {
      if (initObj.hasOwnProperty('read_data')) {
        this.read_data = initObj.read_data
      }
      else {
        this.read_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Modbusreaddata
    // Serialize message field [read_data]
    bufferOffset = _arraySerializer.int32(obj.read_data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Modbusreaddata
    let len;
    let data = new Modbusreaddata(null);
    // Deserialize message field [read_data]
    data.read_data = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.read_data.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Modbusreaddata';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85133f00e9641b3d842dc178852ca264';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] read_data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Modbusreaddata(null);
    if (msg.read_data !== undefined) {
      resolved.read_data = msg.read_data;
    }
    else {
      resolved.read_data = []
    }

    return resolved;
    }
};

module.exports = Modbusreaddata;
