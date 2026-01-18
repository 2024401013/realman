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

class UpdateTCPmasterparam {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.master_name = null;
      this.new_name = null;
      this.ip = null;
      this.port = null;
    }
    else {
      if (initObj.hasOwnProperty('master_name')) {
        this.master_name = initObj.master_name
      }
      else {
        this.master_name = '';
      }
      if (initObj.hasOwnProperty('new_name')) {
        this.new_name = initObj.new_name
      }
      else {
        this.new_name = '';
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UpdateTCPmasterparam
    // Serialize message field [master_name]
    bufferOffset = _serializer.string(obj.master_name, buffer, bufferOffset);
    // Serialize message field [new_name]
    bufferOffset = _serializer.string(obj.new_name, buffer, bufferOffset);
    // Serialize message field [ip]
    bufferOffset = _serializer.string(obj.ip, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.int32(obj.port, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateTCPmasterparam
    let len;
    let data = new UpdateTCPmasterparam(null);
    // Deserialize message field [master_name]
    data.master_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [new_name]
    data.new_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ip]
    data.ip = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.master_name);
    length += _getByteLength(object.new_name);
    length += _getByteLength(object.ip);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/UpdateTCPmasterparam';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76137ea31de5ca5b7a105efb1f13f4c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string master_name #要修改的tcp主站名称
    string new_name #新名称
    string ip #新IP
    int32 port #新端口
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UpdateTCPmasterparam(null);
    if (msg.master_name !== undefined) {
      resolved.master_name = msg.master_name;
    }
    else {
      resolved.master_name = ''
    }

    if (msg.new_name !== undefined) {
      resolved.new_name = msg.new_name;
    }
    else {
      resolved.new_name = ''
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

    return resolved;
    }
};

module.exports = UpdateTCPmasterparam;
