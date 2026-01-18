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

class Gettrajectorylist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.page_num = null;
      this.page_size = null;
      this.vague_search = null;
    }
    else {
      if (initObj.hasOwnProperty('page_num')) {
        this.page_num = initObj.page_num
      }
      else {
        this.page_num = 0;
      }
      if (initObj.hasOwnProperty('page_size')) {
        this.page_size = initObj.page_size
      }
      else {
        this.page_size = 0;
      }
      if (initObj.hasOwnProperty('vague_search')) {
        this.vague_search = initObj.vague_search
      }
      else {
        this.vague_search = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gettrajectorylist
    // Serialize message field [page_num]
    bufferOffset = _serializer.int32(obj.page_num, buffer, bufferOffset);
    // Serialize message field [page_size]
    bufferOffset = _serializer.int32(obj.page_size, buffer, bufferOffset);
    // Serialize message field [vague_search]
    bufferOffset = _serializer.string(obj.vague_search, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gettrajectorylist
    let len;
    let data = new Gettrajectorylist(null);
    // Deserialize message field [page_num]
    data.page_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [page_size]
    data.page_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vague_search]
    data.vague_search = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.vague_search);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rm_msgs/Gettrajectorylist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbe34dfd8b66421bedf70042e463f5a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 page_num
    int32 page_size
    string vague_search
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Gettrajectorylist(null);
    if (msg.page_num !== undefined) {
      resolved.page_num = msg.page_num;
    }
    else {
      resolved.page_num = 0
    }

    if (msg.page_size !== undefined) {
      resolved.page_size = msg.page_size;
    }
    else {
      resolved.page_size = 0
    }

    if (msg.vague_search !== undefined) {
      resolved.vague_search = msg.vague_search;
    }
    else {
      resolved.vague_search = ''
    }

    return resolved;
    }
};

module.exports = Gettrajectorylist;
